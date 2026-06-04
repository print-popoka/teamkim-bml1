"""Main run loop for the maze robot.

Usage:
    python main.py                  # full run, ctrl+c to stop
    python main.py --dry-run        # no GPIO; logic-only smoke test (works off Pi)
    python main.py --duration 30    # auto-stop after 30 seconds
    python main.py --name run-01    # name the trace file
    python main.py --no-camera      # skip camera; signal forced UNKNOWN

The loop:
    1. Poll ultrasonics (front, left45, right45) with median filtering.
    2. Capture a camera frame, run the traffic-light detector.
    3. Feed (distances, signal) into the state machine.
    4. Execute the resulting HighLevelCommand on Motors.
    5. Trace every step into ``logs/runs/*.jsonl``.

Architecture map (per CLAUDE.md):
    hal/         - hardware-touching code (Ultrasonics, Motors)
    perception/  - sensor -> meaning (TrafficLightDetector)
    control/     - smooth wall-follow PD controller
    algorithm/   - top-level state machine (WallFollowerSM)
    logs/        - JSONL trace + replay
"""

from __future__ import annotations

import argparse
import sys
import time

from logs.trace import tracer

LOOP_HZ = 10
LOOP_DT = 1.0 / LOOP_HZ

# Camera doesn't need to run every loop — traffic-light state changes
# slowly (operator-controlled, not flickering). Polling every CAMERA_EVERY
# ticks frees up CPU for the sensor + control loop, which DOES need to
# run at full rate. The last-seen signal persists between camera frames.
CAMERA_EVERY = 3   # 10Hz / 3 = ~3.3Hz camera

# Log loop duration every TICK_LOG_EVERY ticks. Helps us spot when the
# loop falls behind its 100ms budget on the real Pi.
TICK_LOG_EVERY = 30


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Maze robot main loop")
    p.add_argument("--dry-run", action="store_true", help="no hardware; logic-only")
    p.add_argument("--duration", type=float, default=None, help="auto-stop after N seconds")
    p.add_argument("--name", default="run", help="trace file basename")
    p.add_argument("--no-camera", action="store_true", help="skip camera (signal=UNKNOWN)")
    return p.parse_args()


def main() -> int:
    args = parse_args()

    from algorithm.wall_follower_sm import WallFollowerSM
    from hal.motors import Motors

    motors = Motors(dry_run=args.dry_run)
    sm = WallFollowerSM()

    ultras = None
    detector = None
    picam = None

    if not args.dry_run:
        from hal.ultrasonics import Ultrasonics
        ultras = Ultrasonics()
        ultras.setup()

        if not args.no_camera:
            from perception.traffic_light import TrafficLightDetector
            from picamera2 import Picamera2  # type: ignore[import-not-found]
            picam = Picamera2()
            picam.preview_configuration.main.size = (640, 480)
            picam.preview_configuration.main.format = "RGB888"
            picam.configure("preview")
            picam.start()
            time.sleep(1.0)
            detector = TrafficLightDetector()

    motors.setup()

    tracer.start(args.name)
    tracer.info(
        "boot",
        dry_run=args.dry_run,
        camera=not args.no_camera,
        duration=args.duration,
    )

    t0 = time.monotonic()
    frame_id = 0
    tick_count = 0
    last_signal: str = "UNKNOWN"
    try:
        while True:
            tick_start = time.monotonic()
            elapsed = tick_start - t0
            if args.duration is not None and elapsed >= args.duration:
                tracer.info("duration reached, exiting", elapsed_s=elapsed)
                break

            # Sensors (every tick — these drive wall-following).
            if ultras is not None:
                distances = ultras.poll_all(["front", "left45", "right45"])
                f, l, r = distances["front"], distances["left45"], distances["right45"]
            else:
                f = l = r = None  # dry-run

            # Perception (every CAMERA_EVERY ticks). last_signal persists
            # between captures, so the state machine still sees the most
            # recent decision on the non-capture ticks.
            if (
                detector is not None
                and picam is not None
                and tick_count % CAMERA_EVERY == 0
            ):
                frame = picam.capture_array()
                reading = detector.detect(frame, frame_id=frame_id)
                last_signal = reading.signal
                frame_id += 1

            # State machine + motor command
            cmd = sm.step(f, l, r, last_signal)
            _execute(motors, cmd)

            # Periodic loop-duration health check (helps spot the Pi
            # falling behind its 100ms budget).
            tick_dur_ms = (time.monotonic() - tick_start) * 1000.0
            if tick_count % TICK_LOG_EVERY == 0:
                tracer.info("tick_health", dur_ms=round(tick_dur_ms, 1), tick=tick_count)
            tick_count += 1

            slept = LOOP_DT - tick_dur_ms / 1000.0
            if slept > 0:
                time.sleep(slept)

    except KeyboardInterrupt:
        tracer.info("KeyboardInterrupt")
    finally:
        motors.stop()
        motors.cleanup()
        if ultras is not None:
            ultras.cleanup()
        if picam is not None:
            try:
                picam.stop()
            except Exception:  # noqa: BLE001
                pass
        if not args.dry_run:
            try:
                import RPi.GPIO as _GPIO
                _GPIO.cleanup()
            except Exception:  # noqa: BLE001
                pass
        tracer.stop()

    return 0


def _execute(motors, cmd) -> None:
    if cmd.action == "stop":
        motors.stop()
    elif cmd.action == "forward":
        motors.forward(cmd.linear_speed)
    elif cmd.action == "backward":
        motors.backward(cmd.linear_speed)
    elif cmd.action == "arc":
        motors.arc(cmd.linear_speed, cmd.curvature)
    elif cmd.action == "pivot_right":
        motors.pivot_right(cmd.linear_speed)
    elif cmd.action == "pivot_left":
        motors.pivot_left(cmd.linear_speed)
    else:
        motors.stop()


if __name__ == "__main__":
    sys.exit(main())
