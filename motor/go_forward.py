"""Standalone forward test primitive — visual verification of forward movement.

Open-loop time-based forward using the Motors class helper. Run this, watch the car
drive forward in a straight line, and check if it drifts or goes straight.

Usage on the Pi:
    python motor/go_forward.py                 # forward at PWM 45 for 1.0s
    python motor/go_forward.py --pwm 50        # override duty cycle
    python motor/go_forward.py --seconds 1.5   # override duration

Off-Pi (dev machine, no RPi.GPIO):
    python motor/go_forward.py --dry-run       # logs only, no wheel motion
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from hal.motors import Motors  # noqa: E402


DEFAULT_PWM = 45.0
DEFAULT_SECONDS = 0.3


def drive_forward(motors: Motors, pwm: float, seconds: float) -> None:
    """Drive forward for `seconds`, then hard stop. Blocks until done."""
    print(f"[go_forward] FORWARD  pwm={pwm:.0f}  seconds={seconds:.3f}")
    motors.forward(pwm)
    time.sleep(seconds)
    motors.stop()


def main() -> int:
    ap = argparse.ArgumentParser(description="Open-loop forward drive test")
    ap.add_argument(
        "--pwm",
        type=float,
        default=DEFAULT_PWM,
        help=f"duty cycle for forward drive (default {DEFAULT_PWM:.0f})",
    )
    ap.add_argument(
        "--seconds",
        type=float,
        default=DEFAULT_SECONDS,
        help=f"duration in seconds (default {DEFAULT_SECONDS:.3f}s)",
    )
    ap.add_argument(
        "--dry-run",
        action="store_true",
        help="no hardware; logs only (for off-Pi sanity checks)",
    )
    ap.add_argument(
        "--no-prompt",
        action="store_true",
        help="skip the ENTER prompt before driving",
    )
    args = ap.parse_args()

    motors = Motors(dry_run=args.dry_run)
    motors.setup()

    try:
        if not args.no_prompt:
            print("[go_forward] place car on clear floor (≥ 1.5m straight path)")
            input("[go_forward] ENTER to start... ")

        drive_forward(motors, args.pwm, args.seconds)

    finally:
        motors.stop()
        motors.cleanup()

    print("[go_forward] done")
    return 0


if __name__ == "__main__":
    sys.exit(main())
