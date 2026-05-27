"""Standalone 90° turn primitive — visual verification of turn calibration.

Open-loop time-based pivot using the calibration constant
``TURN_DEG_PER_SEC_AT_50`` in ``hal/motors.py``. Run this, watch the car
turn, measure with a protractor / phone app, adjust the constant.

Once the angle is correct, the same primitive (``Motors.pivot_left/right``
+ a known duration) can be wired into the state machine.

Usage on the Pi:
    python motor/turn_90.py left              # 90° left at PWM 50
    python motor/turn_90.py right             # 90° right
    python motor/turn_90.py both              # left then 1s pause then right
    python motor/turn_90.py left --pwm 60     # override duty cycle
    python motor/turn_90.py left --seconds 0.8  # override duration

Off-Pi (dev machine, no RPi.GPIO):
    python motor/turn_90.py left --dry-run    # logs only, no wheel motion

Place the car in a clear space (≥ 1 m radius) before running.

This module deliberately does NOT redefine the pin map or GPIO setup —
it imports from hal/motors so the source of truth stays in one place.
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from hal.motors import Motors, TURN_DEG_PER_SEC_AT_50  # noqa: E402


DEFAULT_PWM = 50.0


def _default_seconds() -> float:
    """How long to spin at PWM 50 to cover 90°, per the calibration constant."""
    return 90.0 / TURN_DEG_PER_SEC_AT_50


def turn(motors: Motors, direction: str, pwm: float, seconds: float) -> None:
    """Pivot in place for `seconds`, then hard stop. Blocks until done."""
    print(f"[turn_90] {direction.upper():5}  pwm={pwm:.0f}  seconds={seconds:.3f}")
    if direction == "left":
        motors.pivot_left(pwm)
    elif direction == "right":
        motors.pivot_right(pwm)
    else:
        raise ValueError(f"direction must be 'left' or 'right', got {direction!r}")
    time.sleep(seconds)
    motors.stop()


def main() -> int:
    ap = argparse.ArgumentParser(description="Open-loop 90° pivot")
    ap.add_argument(
        "direction",
        choices=("left", "right", "both"),
        help="'both' = left turn, 1s pause, right turn",
    )
    ap.add_argument(
        "--pwm",
        type=float,
        default=DEFAULT_PWM,
        help=f"duty cycle for the turn (default {DEFAULT_PWM:.0f})",
    )
    ap.add_argument(
        "--seconds",
        type=float,
        default=None,
        help=(
            "override duration in seconds "
            f"(default = 90 / TURN_DEG_PER_SEC_AT_50 = {_default_seconds():.3f}s)"
        ),
    )
    ap.add_argument(
        "--dry-run",
        action="store_true",
        help="no hardware; logs only (for off-Pi sanity checks)",
    )
    ap.add_argument(
        "--no-prompt",
        action="store_true",
        help="skip the ENTER prompt before turning",
    )
    args = ap.parse_args()

    seconds = args.seconds if args.seconds is not None else _default_seconds()

    motors = Motors(dry_run=args.dry_run)
    motors.setup()

    try:
        if not args.no_prompt:
            print("[turn_90] place car in clear space (≥ 1m radius)")
            input("[turn_90] ENTER to start... ")

        if args.direction == "both":
            turn(motors, "left", args.pwm, seconds)
            print("[turn_90] pausing 1s before right turn...")
            time.sleep(1.0)
            turn(motors, "right", args.pwm, seconds)
        else:
            turn(motors, args.direction, args.pwm, seconds)

    finally:
        motors.stop()
        motors.cleanup()

    print("[turn_90] done")
    return 0


if __name__ == "__main__":
    sys.exit(main())
