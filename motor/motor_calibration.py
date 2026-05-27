"""Interactive motor calibration — run on the Pi at the sample maze.

You drive each test, the car moves, you measure with a ruler/protractor,
type in the number; the tool computes mean/stdev/asymmetry and prints a
copy-pasteable CALIBRATION block at the end.

Tests, in recommended order on test day:
  1. Minimum start PWM  (LEFT, then RIGHT wheel separately)
  2. Speed table        (forward 1s at PWM 30/50/70/90, measure cm)
  3. Straight drift     (forward 2s at PWM 50, measure lateral cm)
  4. 90 deg turn time   (right-turn-in-place, measure angle)

Usage:
  python motor/motor_calibration.py                  # all tests, 3 trials each
  python motor/motor_calibration.py --trials 5       # more trials per test
  python motor/motor_calibration.py --skip turn drift # skip some tests

Safety:
  - Place the car on a CLEAR floor (no obstacles within ~1m).
  - For turn test, lift the car so wheels spin in free air, OR place on
    a slippery surface so it can rotate cleanly.
  - Tool issues a `stop_all` at every error path and on Ctrl+C.

Pin map MUST match motor/motor.py (manually duplicated; centralize later).
"""

from __future__ import annotations

import argparse
import math
import statistics
import sys
import time

try:
    import RPi.GPIO as GPIO
except ImportError:
    GPIO = None

# ---------------------------------------------------------------------- #
# Pin map  --  must match motor/motor.py exactly.
# ---------------------------------------------------------------------- #
IN1, IN2 = 17, 27   # left motor (A) direction
IN3, IN4 = 22, 5    # right motor (B) direction
ENA, ENB = 18, 19   # PWM enables (A=left, B=right)
PWM_FREQ = 1000


def require_gpio() -> None:
    if GPIO is None:
        print("[FAIL] RPi.GPIO is not installed. Run this script on the Raspberry Pi.")
        sys.exit(1)


def positive_int(raw: str) -> int:
    value = int(raw)
    if value < 1:
        raise argparse.ArgumentTypeError("must be >= 1")
    return value


def positive_float(raw: str) -> float:
    value = float(raw)
    if value <= 0:
        raise argparse.ArgumentTypeError("must be > 0")
    return value


def duty_cycle(raw: str) -> int:
    value = int(raw)
    if not 0 <= value <= 100:
        raise argparse.ArgumentTypeError("must be between 0 and 100")
    return value


# ---------------------------------------------------------------------- #
# Low-level motor primitives
# ---------------------------------------------------------------------- #
def setup() -> tuple[GPIO.PWM, GPIO.PWM]:
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    for p in [IN1, IN2, IN3, IN4, ENA, ENB]:
        GPIO.setup(p, GPIO.OUT)
    pwm_a = GPIO.PWM(ENA, PWM_FREQ)
    pwm_b = GPIO.PWM(ENB, PWM_FREQ)
    pwm_a.start(0)
    pwm_b.start(0)
    return pwm_a, pwm_b


def stop_all(pwm_a: GPIO.PWM, pwm_b: GPIO.PWM) -> None:
    pwm_a.ChangeDutyCycle(0)
    pwm_b.ChangeDutyCycle(0)
    for p in [IN1, IN2, IN3, IN4]:
        GPIO.output(p, False)


def _set_left(forward: bool) -> None:
    GPIO.output(IN1, forward)
    GPIO.output(IN2, not forward)


def _set_right(forward: bool) -> None:
    GPIO.output(IN3, forward)
    GPIO.output(IN4, not forward)


def run_forward(pwm_a: GPIO.PWM, pwm_b: GPIO.PWM, duty: float, seconds: float) -> None:
    _set_left(True)
    _set_right(True)
    pwm_a.ChangeDutyCycle(duty)
    pwm_b.ChangeDutyCycle(duty)
    time.sleep(seconds)
    stop_all(pwm_a, pwm_b)


def turn_right_in_place(pwm_a: GPIO.PWM, pwm_b: GPIO.PWM, duty: float, seconds: float) -> None:
    """Right turn: left wheel forward, right wheel reverse."""
    _set_left(True)
    _set_right(False)
    pwm_a.ChangeDutyCycle(duty)
    pwm_b.ChangeDutyCycle(duty)
    time.sleep(seconds)
    stop_all(pwm_a, pwm_b)


# ---------------------------------------------------------------------- #
# Input helpers
# ---------------------------------------------------------------------- #
def _ask_float(prompt: str) -> float | None:
    raw = input(prompt).strip()
    try:
        return float(raw)
    except ValueError:
        print(f"    [skip] not a number: {raw!r}")
        return None


def _ask_int(prompt: str) -> int | None:
    raw = input(prompt).strip()
    try:
        return int(raw)
    except ValueError:
        print(f"    [skip] not an integer: {raw!r}")
        return None


def _stats(values: list[float]) -> tuple[int, float, float, float, float]:
    """Returns (n, mean, stdev, min, max). All zeros if list is empty."""
    if not values:
        return 0, 0.0, 0.0, 0.0, 0.0
    if len(values) == 1:
        v = values[0]
        return 1, v, 0.0, v, v
    return (
        len(values),
        statistics.mean(values),
        statistics.pstdev(values),
        min(values),
        max(values),
    )


# ---------------------------------------------------------------------- #
# Test 1: minimum start PWM
# ---------------------------------------------------------------------- #
PWM_SWEEP = [5, 8, 10, 12, 15, 18, 20, 25, 30, 40]


def test_min_start_pwm(
    pwm_a: GPIO.PWM,
    pwm_b: GPIO.PWM,
    side: str,
    trials: int,
) -> list[int]:
    """Pulse each PWM in PWM_SWEEP for 0.5s; user reports smallest that moved."""
    print(f"\n[TEST 1/4] Minimum start PWM -- {side.upper()} wheel ({trials} trials)")
    print("    Each trial I pulse PWMs in order:")
    print(f"      {PWM_SWEEP}")
    print("    Watch the wheel; note the SMALLEST PWM that produced any movement.")
    print("    (Lift the OTHER side of the car if needed so only one wheel touches.)")

    measurements: list[int] = []
    for trial in range(trials):
        input(f"\n  Trial {trial + 1}/{trials}: ready? press ENTER...")
        if side == "left":
            _set_left(True)
            pwm = pwm_a
        else:
            _set_right(True)
            pwm = pwm_b
        for duty in PWM_SWEEP:
            print(f"    PWM={duty:3d}%", end="", flush=True)
            pwm.ChangeDutyCycle(duty)
            time.sleep(0.5)
            pwm.ChangeDutyCycle(0)
            time.sleep(0.3)
            print(" .. stopped")
        stop_all(pwm_a, pwm_b)
        v = _ask_int("  Smallest PWM that moved the wheel (e.g., 12): ")
        if v is not None:
            measurements.append(v)
    return measurements


# ---------------------------------------------------------------------- #
# Test 2: speed table (cm per second at fixed PWMs)
# ---------------------------------------------------------------------- #
def test_speed_table(
    pwm_a: GPIO.PWM,
    pwm_b: GPIO.PWM,
    duties: tuple[int, ...],
    trials: int,
    duration: float,
) -> dict[int, list[float]]:
    print(f"\n[TEST 2/4] Speed table -- forward {duration}s per PWM ({trials} trials each)")
    print("    Mark START position with tape; after the run, measure how far")
    print("    the car traveled forward.")

    results: dict[int, list[float]] = {}
    for duty in duties:
        per_duty: list[float] = []
        print(f"\n  PWM = {duty}%")
        for trial in range(trials):
            input(f"    Trial {trial + 1}/{trials}: car at start, ENTER to run {duration}s...")
            run_forward(pwm_a, pwm_b, duty, duration)
            v = _ask_float("    Distance traveled (cm): ")
            if v is not None:
                per_duty.append(v)
        results[duty] = per_duty
    return results


# ---------------------------------------------------------------------- #
# Test 3: straight drift (asymmetry)
# ---------------------------------------------------------------------- #
def test_drift(
    pwm_a: GPIO.PWM,
    pwm_b: GPIO.PWM,
    duty: int,
    trials: int,
    duration: float,
) -> tuple[list[float], list[float]]:
    print(f"\n[TEST 3/4] Straight drift -- PWM {duty}% for {duration}s ({trials} trials)")
    print("    Mark a STRAIGHT reference line on the floor at start.")
    print("    Forward distance: how far ahead the car ended.")
    print("    Lateral drift  : how far sideways from the reference line.")
    print("        + = drifted RIGHT (= LEFT wheel is stronger)")
    print("        - = drifted LEFT  (= RIGHT wheel is stronger)")

    fwds: list[float] = []
    drifts: list[float] = []
    for trial in range(trials):
        input(f"\n  Trial {trial + 1}/{trials}: car at start, ENTER to run...")
        run_forward(pwm_a, pwm_b, duty, duration)
        f = _ask_float("    Forward distance (cm): ")
        d = _ask_float("    Lateral drift (+right, -left) (cm): ")
        if f is not None and d is not None:
            fwds.append(f)
            drifts.append(d)
    return fwds, drifts


# ---------------------------------------------------------------------- #
# Test 4: 90 deg turn time
# ---------------------------------------------------------------------- #
TURN_DURATIONS = (0.4, 0.6, 0.8)


def test_turn_time(
    pwm_a: GPIO.PWM,
    pwm_b: GPIO.PWM,
    duty: int,
    trials: int,
) -> list[float]:
    """Run right-turn-in-place for several durations; user reports angle."""
    print(f"\n[TEST 4/4] Turn rate -- PWM {duty}% right-turn-in-place")
    print(f"    Will run for {list(TURN_DURATIONS)}s; you estimate the angle each time.")
    print("    Tip: chalk-mark a reference direction; use a phone protractor.")

    deg_per_sec: list[float] = []
    for t in TURN_DURATIONS:
        print(f"\n  Duration = {t}s")
        for trial in range(trials):
            input(f"    Trial {trial + 1}/{trials}: car in open space, ENTER to spin {t}s...")
            turn_right_in_place(pwm_a, pwm_b, duty, t)
            ang = _ask_float("    Angle turned (degrees, approx): ")
            if ang is not None and t > 0:
                deg_per_sec.append(ang / t)
    return deg_per_sec


# ---------------------------------------------------------------------- #
# Summary
# ---------------------------------------------------------------------- #
def print_summary(
    min_left: list[int],
    min_right: list[int],
    speed_table: dict[int, list[float]],
    fwd_dists: list[float],
    drifts: list[float],
    deg_per_sec: list[float],
    duration_speed_s: float,
    duration_drift_s: float,
) -> None:
    print("\n" + "=" * 64)
    print("  MOTOR CALIBRATION SUMMARY")
    print("=" * 64)

    # min start PWM ------------------------------------------------------
    if min_left:
        n, m, sd, lo, hi = _stats([float(x) for x in min_left])
        print(f"\n  MIN_PWM_LEFT  = {round(m):3d}    n={n}  stdev={sd:.1f}  range {lo:.0f}-{hi:.0f}")
    if min_right:
        n, m, sd, lo, hi = _stats([float(x) for x in min_right])
        print(f"  MIN_PWM_RIGHT = {round(m):3d}    n={n}  stdev={sd:.1f}  range {lo:.0f}-{hi:.0f}")

    # speed table --------------------------------------------------------
    if speed_table:
        print(f"\n  SPEED TABLE  (forward {duration_speed_s}s per measurement)")
        for duty, dists in sorted(speed_table.items()):
            n, m, sd, lo, hi = _stats(dists)
            speed = m / duration_speed_s if duration_speed_s else 0.0
            print(
                f"    PWM {duty:3d}%  ->  {speed:6.1f} cm/s   "
                f"(mean dist {m:5.1f}cm, n={n}, stdev {sd:.2f})"
            )

    # drift / asymmetry --------------------------------------------------
    n_f, m_f, _, _, _ = _stats(fwd_dists)
    n_d, m_d, sd_d, _, _ = _stats(drifts)
    if n_f and n_d and m_f > 0:
        ang_per_sec = math.degrees(math.atan2(m_d, m_f)) / duration_drift_s
        if m_d > 0.5:
            who = "LEFT wheel stronger -> reduce LEFT PWM or boost RIGHT"
        elif m_d < -0.5:
            who = "RIGHT wheel stronger -> reduce RIGHT PWM or boost LEFT"
        else:
            who = "balanced"
        corr_pct = abs(m_d) / m_f * 100.0
        print(
            f"\n  STRAIGHT DRIFT (PWM 50%): {m_d:+.1f}cm sideways over {m_f:.1f}cm forward"
        )
        print(f"    -> {ang_per_sec:+.1f} deg/sec angular drift")
        print(f"    -> {who}  (~{corr_pct:.0f}% PWM trim)")

    # turn rate ----------------------------------------------------------
    n, m, sd, lo, hi = _stats(deg_per_sec)
    if n and m > 0:
        sec_per_90 = 90.0 / m
        print(f"\n  TURN RATE (PWM 50% in-place): {m:.0f} deg/s  (n={n}, stdev {sd:.1f})")
        print(f"    -> 90 deg ~ {sec_per_90:.2f} sec")

    # paste block --------------------------------------------------------
    print("\n" + "=" * 64)
    print("  COPY-PASTE INTO CODE (motor/motor.py or future hal/motors.py):")
    print("=" * 64)
    print(f"\n# Motor calibration measured {time.strftime('%Y-%m-%d')}")
    print("# Surface: <sample maze 우드락 floor>")
    print("# Battery: <state of charge>")
    if min_left:
        print(f"MIN_PWM_LEFT  = {round(_stats([float(x) for x in min_left])[1])}")
    if min_right:
        print(f"MIN_PWM_RIGHT = {round(_stats([float(x) for x in min_right])[1])}")
    if speed_table:
        print("SPEED_CM_PER_SEC = {")
        for duty, dists in sorted(speed_table.items()):
            m_d = _stats(dists)[1]
            speed = m_d / duration_speed_s if duration_speed_s else 0.0
            print(f"    {duty}: {speed:.1f},")
        print("}")
    if deg_per_sec:
        m = _stats(deg_per_sec)[1]
        print(f"TURN_DEG_PER_SEC = {m:.1f}")
        print(f"TURN_SEC_PER_90  = {90.0 / m:.3f}")
    if n_f and n_d and m_f > 0:
        print(f"DRIFT_LATERAL_CM_PER_FORWARD_CM = {m_d / m_f:+.4f}")
    print()


# ---------------------------------------------------------------------- #
# Main
# ---------------------------------------------------------------------- #
def main() -> None:
    ap = argparse.ArgumentParser(description="Interactive motor calibration")
    ap.add_argument(
        "--skip",
        nargs="*",
        default=[],
        choices=["minpwm", "speed", "drift", "turn"],
        help="Skip tests by name",
    )
    ap.add_argument("--trials", type=positive_int, default=3, help="Trials per measurement (default 3)")
    ap.add_argument("--speed-duration", type=positive_float, default=1.0)
    ap.add_argument("--drift-duration", type=positive_float, default=2.0)
    ap.add_argument("--drift-pwm", type=duty_cycle, default=50)
    ap.add_argument("--turn-pwm", type=duty_cycle, default=50)
    ap.add_argument(
        "--speed-pwms",
        type=duty_cycle,
        nargs="+",
        default=[30, 50, 70, 90],
        help="PWM duties to test in speed table",
    )
    args = ap.parse_args()
    require_gpio()

    pwm_a, pwm_b = setup()

    min_left: list[int] = []
    min_right: list[int] = []
    speed_table: dict[int, list[float]] = {}
    fwd_dists: list[float] = []
    drifts: list[float] = []
    deg_per_sec: list[float] = []

    try:
        if "minpwm" not in args.skip:
            min_left = test_min_start_pwm(pwm_a, pwm_b, "left", args.trials)
            min_right = test_min_start_pwm(pwm_a, pwm_b, "right", args.trials)
        if "speed" not in args.skip:
            speed_table = test_speed_table(
                pwm_a, pwm_b, tuple(args.speed_pwms), args.trials, args.speed_duration
            )
        if "drift" not in args.skip:
            fwd_dists, drifts = test_drift(
                pwm_a, pwm_b, args.drift_pwm, args.trials, args.drift_duration
            )
        if "turn" not in args.skip:
            deg_per_sec = test_turn_time(pwm_a, pwm_b, args.turn_pwm, args.trials)

        print_summary(
            min_left,
            min_right,
            speed_table,
            fwd_dists,
            drifts,
            deg_per_sec,
            args.speed_duration,
            args.drift_duration,
        )

    except KeyboardInterrupt:
        print("\n[INTERRUPTED]")
    finally:
        stop_all(pwm_a, pwm_b)
        pwm_a.stop()
        pwm_b.stop()
        GPIO.cleanup()


if __name__ == "__main__":
    main()
