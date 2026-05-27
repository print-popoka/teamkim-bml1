"""HC-SR04 noise characterization tool.

Run for N samples at a fixed (known) distance, collect raw readings,
print statistics. Use this to figure out:
  - Noise floor (stddev)
  - Bias (mean - true distance)
  - Fail rate (timeouts)
  - Linearity across distances (run at several true distances and compare)

Why a separate tool?
  `sensor/ultrasonic.py` is the lecture baseline (infinite loop, no stats).
  This one returns stats and exits, so it's measurement-grade.

Usage examples:
  # FRONT sensor (default pins 23/24), 200 samples, label true distance
  python sensor/ultrasonic_noise.py --true 20

  # LEFT45 sensor on recommended pins
  python sensor/ultrasonic_noise.py --trig 25 --echo 8 --n 200 --true 20

  # No-wall baseline (no --true), see how often timeout fires
  python sensor/ultrasonic_noise.py --n 100
"""

import argparse
import statistics
import sys
import time

try:
    import RPi.GPIO as GPIO
except ImportError:
    GPIO = None


def measure_once(trig, echo, timeout=0.015):
    """Single HC-SR04 ping. Returns distance in cm, or None on timeout."""
    GPIO.output(trig, False)
    time.sleep(0.002)

    GPIO.output(trig, True)
    time.sleep(0.00001)
    GPIO.output(trig, False)

    start_wait = time.perf_counter()
    while GPIO.input(echo) == 0:
        if time.perf_counter() - start_wait > timeout:
            return None

    pulse_start = time.perf_counter()
    while GPIO.input(echo) == 1:
        if time.perf_counter() - pulse_start > timeout:
            return None
    pulse_end = time.perf_counter()

    # 34300 cm/s, divide by 2 for round-trip == multiply by 17150
    return (pulse_end - pulse_start) * 17150


def positive_int(raw):
    value = int(raw)
    if value < 1:
        raise argparse.ArgumentTypeError("must be >= 1")
    return value


def non_negative_float(raw):
    value = float(raw)
    if value < 0:
        raise argparse.ArgumentTypeError("must be >= 0")
    return value


def require_gpio():
    if GPIO is None:
        print("[FAIL] RPi.GPIO is not installed. Run this script on the Raspberry Pi.")
        sys.exit(1)


def main():
    ap = argparse.ArgumentParser(description="HC-SR04 noise characterization")
    ap.add_argument("--trig", type=int, default=23, help="TRIG BCM pin (default 23 = FRONT)")
    ap.add_argument("--echo", type=int, default=24, help="ECHO BCM pin (default 24 = FRONT)")
    ap.add_argument("--n", type=positive_int, default=200, help="number of samples (default 200)")
    ap.add_argument(
        "--interval",
        type=non_negative_float,
        default=0.04,
        help="seconds between pings (default 0.04 = 25Hz, above 10ms cooldown)",
    )
    ap.add_argument("--timeout", type=non_negative_float, default=0.015)
    ap.add_argument(
        "--true",
        dest="true_cm",
        type=float,
        default=None,
        help="ground-truth distance in cm (enables bias calculation)",
    )
    ap.add_argument("--quiet", action="store_true", help="suppress per-sample print")
    ap.add_argument(
        "--warmup",
        type=int,
        default=2,
        help="discard this many initial pings (HC-SR04 first ping is noisy; default 2)",
    )
    args = ap.parse_args()
    require_gpio()

    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    GPIO.setup(args.trig, GPIO.OUT)
    GPIO.setup(args.echo, GPIO.IN)
    GPIO.output(args.trig, False)
    time.sleep(0.2)

    print(f"[CONFIG] TRIG={args.trig} ECHO={args.echo} n={args.n} interval={args.interval}s")
    if args.true_cm is not None:
        print(f"[CONFIG] True distance: {args.true_cm} cm")
    print()

    # Warmup pings — discarded. HC-SR04 first ping after idle reads as a
    # large outlier (capacitor settling).
    for _ in range(max(0, args.warmup)):
        measure_once(args.trig, args.echo)
        time.sleep(args.interval)

    samples = []
    fails = 0

    try:
        for i in range(args.n):
            d = measure_once(args.trig, args.echo, args.timeout)
            if d is None:
                fails += 1
                if not args.quiet:
                    print(f"  {i:3d}: FAIL")
            else:
                samples.append(d)
                if not args.quiet:
                    print(f"  {i:3d}: {d:6.2f} cm")
            time.sleep(args.interval)
    finally:
        GPIO.cleanup()

    print()
    print("=" * 52)
    print(f"  Valid samples : {len(samples)} / {args.n}")
    print(f"  Fails         : {fails} ({100 * fails / args.n:.1f}%)")
    if samples:
        mean = statistics.mean(samples)
        median = statistics.median(samples)
        stddev = statistics.pstdev(samples)
        print(f"  Mean          : {mean:6.2f} cm")
        print(f"  Median        : {median:6.2f} cm")
        print(f"  Stddev        : {stddev:6.2f} cm")
        print(f"  Min           : {min(samples):6.2f} cm")
        print(f"  Max           : {max(samples):6.2f} cm")
        print(f"  Range (max-min): {max(samples) - min(samples):6.2f} cm")
        if args.true_cm is not None:
            print(f"  Bias (mean-true): {mean - args.true_cm:+6.2f} cm")
    print("=" * 52)


if __name__ == "__main__":
    main()
