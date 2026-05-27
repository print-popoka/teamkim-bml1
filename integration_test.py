"""End-to-end integration test: forward + obstacle U-turn + traffic-light response.

This is **not** the maze-solving algorithm — it's a deterministic
hardware-level integration test you can run from the autostart unit:

    1. Drive FORWARD continuously
    2. FRONT < EMERGENCY_CM (default 3cm) -> immediate STOP + EXIT
    3. FRONT <= OBSTACLE_CM (default 10cm), only the FIRST time:
         -> in-place U-turn (180°), then resume forward
       Subsequent obstacles -> HALTED state (motors stopped, waiting)
    4. Camera RED  -> in-place right turn (90°), resume forward
       (ignored in HALTED state)
    5. Camera GREEN -> graceful exit (test PASS)

Rising-edge triggering on the traffic-light signal means seeing the same
light over multiple frames doesn't cause repeat turns / repeat exits.

Stop the test (when no green appears):
    - Ctrl+C in the SAME terminal
    - From another terminal:  pkill -f integration_test
                              # or: kill -TERM <pid>
    - Or wait for --duration (default 300s) safety timeout
    - Pull the battery (last resort)

Usage on the Pi:
    python integration_test.py                    # all hardware, defaults
    python integration_test.py --verbose          # live print of camera signal
    python integration_test.py --duration 60      # 1-min cap
    python integration_test.py --no-camera        # obstacle-only
    python integration_test.py --obstacle-cm 15   # tweak trigger distance
    python integration_test.py --emergency-cm 5   # tweak hard-stop distance
    python integration_test.py --dry-run          # off-Pi smoke
"""

from __future__ import annotations

import argparse
import signal
import sys
import time

from hal.motors import Motors, TURN_DEG_PER_SEC_AT_50
from logs.trace import tracer

# Tunables (CLI flags override) ----------------------------------------
DEFAULT_FORWARD_PWM = 40.0
DEFAULT_TURN_PWM = 50.0          # matches turn-rate calibration
DEFAULT_OBSTACLE_CM = 10.0       # was 20; user-tightened so people walking by don't trigger
DEFAULT_EMERGENCY_CM = 3.0       # hard stop: too close to safely U-turn out of
DEFAULT_DURATION_S = 300.0       # 5-min safety cap
LOOP_HZ = 10
LOOP_DT = 1.0 / LOOP_HZ
CAMERA_EVERY = 3                 # poll camera every Nth tick (~3.3 Hz)

# Derived from calibrated turn rate.
SEC_PER_90 = 90.0 / TURN_DEG_PER_SEC_AT_50
SEC_PER_180 = 2.0 * SEC_PER_90


# Signal handler state shared between handler and main loop.
_stop_requested = False


def _request_stop(_signum, _frame) -> None:
    global _stop_requested
    _stop_requested = True


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("--dry-run", action="store_true", help="no hardware; logic only")
    p.add_argument("--no-camera", action="store_true", help="skip camera; signal stays UNKNOWN")
    p.add_argument("--verbose", action="store_true", help="live print of every camera read and state transition")
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
        "--emergency-cm",
        type=float,
        default=DEFAULT_EMERGENCY_CM,
        help=f"front-distance hard-stop threshold (default {DEFAULT_EMERGENCY_CM:g})",
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

    # Graceful stop on SIGTERM (e.g., from `kill <pid>` or `systemctl stop`).
    # SIGINT (Ctrl+C) raises KeyboardInterrupt naturally; we catch it below.
    signal.signal(signal.SIGTERM, _request_stop)

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
        emergency_cm=args.emergency_cm,
        forward_pwm=args.forward_pwm,
        turn_pwm=args.turn_pwm,
        sec_per_90=round(SEC_PER_90, 3),
        sec_per_180=round(SEC_PER_180, 3),
        dry_run=args.dry_run,
        no_camera=args.no_camera,
        duration_cap_s=args.duration,
    )

    exit_reason = "unknown"
    state = "FORWARD"
    uturn_done = False
    last_signal: str = "UNKNOWN"
    prev_signal: str = "UNKNOWN"
    tick = 0

    def _go_state(new_state: str, reason: str) -> None:
        nonlocal state
        tracer.state(state=new_state, from_state=state, reason=reason)
        if args.verbose:
            print(f"[state] {state} -> {new_state}  ({reason})")
        state = new_state

    try:
        motors.forward(args.forward_pwm)
        tracer.state(state="FORWARD", from_state=None, reason="boot")
        if args.verbose:
            print(f"[boot] forward @ {args.forward_pwm:.0f} PWM")

        t0 = time.monotonic()
        while True:
            if _stop_requested:
                exit_reason = "sigterm"
                tracer.info("sigterm_received")
                break

            tick_start = time.monotonic()
            elapsed = tick_start - t0
            if elapsed >= args.duration:
                exit_reason = "safety_timeout"
                tracer.info("safety_timeout", elapsed_s=round(elapsed, 2))
                break

            # ---- FRONT distance ----------------------------------------
            front_cm = ultras.poll("front") if ultras is not None else None

            # ---- 0. EMERGENCY STOP — highest priority, always checked --
            if front_cm is not None and front_cm < args.emergency_cm:
                exit_reason = "emergency_stop"
                tracer.info("emergency_stop_triggered", front_cm=round(front_cm, 2))
                if args.verbose:
                    print(f"[EMERGENCY] front={front_cm:.1f}cm < {args.emergency_cm}cm — STOPPING")
                break

            # ---- Camera (throttled) ------------------------------------
            if detector is not None and picam is not None and tick % CAMERA_EVERY == 0:
                frame = picam.capture_array()
                reading = detector.detect(frame, frame_id=tick)
                last_signal = reading.signal
                if args.verbose:
                    print(
                        f"[cam] signal={last_signal:<7} raw={reading.raw_signal:<7} "
                        f"red={reading.red_area:>5} green={reading.green_area:>5}"
                    )

            # ---- 1. GREEN rising edge — exit (works in any state) -----
            if last_signal == "GO" and prev_signal != "GO":
                exit_reason = "green_seen"
                tracer.info("green_rising_edge_exit", state=state)
                if args.verbose:
                    print("[GREEN] rising edge — exiting")
                break

            # ---- State-dependent reactions -----------------------------
            if state == "HALTED":
                # In HALTED, motors stay stopped. Only GREEN (above) or
                # safety timeout / SIGTERM can exit. RED is logged but
                # ignored — turning into a wall isn't safe.
                if last_signal == "STOP" and prev_signal != "STOP":
                    tracer.info("red_ignored_in_halted")
                    if args.verbose:
                        print("[HALTED] RED detected but ignored (motors won't move)")
            else:  # FORWARD
                # ---- 2. RED rising edge — right turn 90° -----------
                if last_signal == "STOP" and prev_signal != "STOP":
                    _go_state("TURNING_RIGHT_90", "red")
                    motors.pivot_right(args.turn_pwm)
                    time.sleep(SEC_PER_90)
                    motors.forward(args.forward_pwm)
                    _go_state("FORWARD", "turn_done")

                # ---- 3. Obstacle — U-turn (first time only) --------
                elif front_cm is not None and front_cm <= args.obstacle_cm:
                    if not uturn_done:
                        _go_state("U_TURNING_180", f"obstacle_{front_cm:.1f}cm")
                        motors.pivot_right(args.turn_pwm)
                        time.sleep(SEC_PER_180)
                        motors.forward(args.forward_pwm)
                        uturn_done = True
                        _go_state("FORWARD", "uturn_done")
                    else:
                        # 2nd+ obstacle: stop the wheels, sit and wait.
                        motors.stop()
                        _go_state("HALTED", f"obstacle_after_uturn_{front_cm:.1f}cm")

            prev_signal = last_signal
            tick += 1

            slept = LOOP_DT - (time.monotonic() - tick_start)
            if slept > 0:
                time.sleep(slept)

        tracer.info("integration_end", exit_reason=exit_reason, ticks=tick, final_state=state)

    except KeyboardInterrupt:
        exit_reason = "keyboard_interrupt"
        tracer.info("keyboard_interrupt", state=state)
    finally:
        motors.stop()
        motors.cleanup()
        if ultras is not None:
            ultras.cleanup()
        if picam is not None:
            picam.stop()
        tracer.stop()
        print(f"[integration_test] done — exit_reason={exit_reason}  final_state={state}")

    # Exit code: success on green/keyboard/sigterm, failure on timeout/emergency.
    return 0 if exit_reason in ("green_seen", "keyboard_interrupt", "sigterm") else 1


if __name__ == "__main__":
    sys.exit(main())
