"""End-to-end integration test: forward + obstacle U-turn + traffic-light response.

This is **not** the maze-solving algorithm — it's a deterministic
hardware-level integration test you can run from the autostart unit at
the sample maze:

    1. Drive FORWARD continuously
    2. FRONT ultrasonic ≤ OBSTACLE_CM → in-place U-turn (180°), resume forward
    3. Camera sees RED  → in-place right turn (90°), resume forward
    4. Camera sees GREEN → graceful exit (test PASS)

Rising-edge triggering on the traffic-light signal means seeing the same
light over multiple frames doesn't cause repeat turns / repeat exits.

Safety cap: --duration ends the loop even if no green ever appears.

Usage on the Pi:
    python integration_test.py                    # all hardware, 5-min cap
    python integration_test.py --duration 60      # 1-min cap
    python integration_test.py --no-camera        # obstacle-only (signal=UNKNOWN)
    python integration_test.py --obstacle-cm 30   # tweak trigger distance
    python integration_test.py --dry-run          # off-Pi smoke (logic only)
"""

from __future__ import annotations

import argparse
import sys
import time

from hal.motors import Motors, TURN_DEG_PER_SEC_AT_50
from logs.trace import tracer

# Tunables (CLI flags override) ----------------------------------------
DEFAULT_FORWARD_PWM = 40.0
DEFAULT_TURN_PWM = 50.0          # matches turn-rate calibration
DEFAULT_OBSTACLE_CM = 20.0
DEFAULT_DURATION_S = 300.0       # 5-min safety cap
LOOP_HZ = 10
LOOP_DT = 1.0 / LOOP_HZ
CAMERA_EVERY = 3                 # poll camera every Nth tick (~3.3 Hz)

# Derived from calibrated turn rate.
SEC_PER_90 = 90.0 / TURN_DEG_PER_SEC_AT_50
SEC_PER_180 = 2.0 * SEC_PER_90


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("--dry-run", action="store_true", help="no hardware; logic only")
    p.add_argument("--no-camera", action="store_true", help="skip camera; signal stays UNKNOWN")
    p.add_argument(
        "--duration",
        type=float,
        default=DEFAULT_DURATION_S,
        help=f"safety cap in seconds (default {DEFAULT_DURATION_S:g})",
    )
    p.add_argument(
        "--obstacle-cm",
        type=float,
        default=DEFAULT_OBSTACLE_CM,
        help=f"front-distance trigger for U-turn (default {DEFAULT_OBSTACLE_CM:g})",
    )
    p.add_argument(
        "--forward-pwm",
        type=float,
        default=DEFAULT_FORWARD_PWM,
        help=f"forward PWM (default {DEFAULT_FORWARD_PWM:g})",
    )
    p.add_argument(
        "--turn-pwm",
        type=float,
        default=DEFAULT_TURN_PWM,
        help=f"turn PWM (default {DEFAULT_TURN_PWM:g})",
    )
    p.add_argument("--name", default="integration", help="trace file basename")
    return p.parse_args()


def main() -> int:
    args = parse_args()

    # ---- hardware setup ------------------------------------------------
    motors = Motors(dry_run=args.dry_run)
    motors.setup()

    ultras = None
    if not args.dry_run:
        from hal.ultrasonics import Ultrasonics  # local import: Pi-only
        ultras = Ultrasonics()
        ultras.setup()

    detector = None
    picam = None
    if not args.dry_run and not args.no_camera:
        from picamera2 import Picamera2  # local import: Pi-only
        from perception.traffic_light import TrafficLightDetector
        picam = Picamera2()
        picam.preview_configuration.main.size = (640, 480)
        picam.preview_configuration.main.format = "RGB888"
        picam.configure("preview")
        picam.start()
        time.sleep(1.0)
        detector = TrafficLightDetector()

    tracer.start(args.name)
    tracer.info(
        "integration_start",
        obstacle_cm=args.obstacle_cm,
        forward_pwm=args.forward_pwm,
        turn_pwm=args.turn_pwm,
        sec_per_90=round(SEC_PER_90, 3),
        sec_per_180=round(SEC_PER_180, 3),
        dry_run=args.dry_run,
        no_camera=args.no_camera,
        duration_cap_s=args.duration,
    )

    exit_reason = "unknown"
    last_signal: str = "UNKNOWN"
    prev_signal: str = "UNKNOWN"
    tick = 0

    try:
        motors.forward(args.forward_pwm)
        tracer.state(state="FORWARD", from_state=None, reason="boot")

        t0 = time.monotonic()
        while True:
            tick_start = time.monotonic()
            elapsed = tick_start - t0
            if elapsed >= args.duration:
                exit_reason = "safety_timeout"
                tracer.info("safety_timeout", elapsed_s=round(elapsed, 2))
                break

            # ---- FRONT distance ----------------------------------------
            front_cm = ultras.poll("front") if ultras is not None else None

            # ---- Camera (throttled) ------------------------------------
            if detector is not None and picam is not None and tick % CAMERA_EVERY == 0:
                frame = picam.capture_array()
                reading = detector.detect(frame, frame_id=tick)
                last_signal = reading.signal

            # ---- Event handling (priority: GREEN > RED > obstacle) ----
            # Rising edges only — repeated same-state frames don't re-fire.
            if last_signal == "GO" and prev_signal != "GO":
                exit_reason = "green_seen"
                tracer.info("green_rising_edge_exit")
                break

            if last_signal == "STOP" and prev_signal != "STOP":
                tracer.state(state="TURNING_RIGHT_90", from_state="FORWARD", reason="red")
                motors.pivot_right(args.turn_pwm)
                time.sleep(SEC_PER_90)
                motors.forward(args.forward_pwm)
                tracer.state(state="FORWARD", from_state="TURNING_RIGHT_90", reason="turn_done")

            elif front_cm is not None and front_cm <= args.obstacle_cm:
                tracer.state(
                    state="U_TURNING_180",
                    from_state="FORWARD",
                    reason=f"obstacle_{front_cm:.1f}cm",
                )
                motors.pivot_right(args.turn_pwm)
                time.sleep(SEC_PER_180)
                motors.forward(args.forward_pwm)
                tracer.state(state="FORWARD", from_state="U_TURNING_180", reason="uturn_done")

            prev_signal = last_signal
            tick += 1

            slept = LOOP_DT - (time.monotonic() - tick_start)
            if slept > 0:
                time.sleep(slept)

        tracer.info("integration_end", exit_reason=exit_reason, ticks=tick)

    except KeyboardInterrupt:
        exit_reason = "keyboard_interrupt"
        tracer.info("keyboard_interrupt")
    finally:
        motors.stop()
        motors.cleanup()
        if ultras is not None:
            ultras.cleanup()
        if picam is not None:
            picam.stop()
        tracer.stop()
        print(f"[integration_test] done — exit_reason={exit_reason}")

    return 0 if exit_reason in ("green_seen", "keyboard_interrupt") else 1


if __name__ == "__main__":
    sys.exit(main())
