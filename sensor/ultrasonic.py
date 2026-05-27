"""HC-SR04 ultrasonic distance measurement (BML1 lecture).

Wiring (BCM):
    TRIG -> GPIO 23 (output)
    ECHO -> GPIO 24 (input)
    VCC  -> 5V, GND -> GND

Run: python ultrasonic.py     (quit with Ctrl+C)
"""

import argparse
import sys
import time


def non_negative_float(raw):
    value = float(raw)
    if value < 0:
        raise argparse.ArgumentTypeError("must be >= 0")
    return value


def positive_int(raw):
    value = int(raw)
    if value < 1:
        raise argparse.ArgumentTypeError("must be >= 1")
    return value


def parse_args():
    parser = argparse.ArgumentParser(description="HC-SR04 distance loop")
    parser.add_argument("--timeout", type=non_negative_float, default=0.015)
    parser.add_argument("--interval", type=non_negative_float, default=0.2)
    parser.add_argument("--print-every", type=positive_int, default=1)
    parser.add_argument("--quiet-debug", action="store_true")
    return parser.parse_args()


args = parse_args()

try:
    import RPi.GPIO as GPIO
except ImportError:
    print("[FAIL] RPi.GPIO is not installed. Run this script on the Raspberry Pi.")
    sys.exit(1)

GPIO.setmode(GPIO.BCM)

TRIG = 23
ECHO = 24

GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)
GPIO.output(TRIG, False)


def measure_distance(timeout=0.015, debug=True):
    GPIO.output(TRIG, False)
    time.sleep(0.05)

    # Send 10us trigger pulse.
    GPIO.output(TRIG, True)
    time.sleep(0.00001)
    GPIO.output(TRIG, False)

    if debug:
        print("[DEBUG] TRIG pulse sent")

    start_wait = time.perf_counter()
    while GPIO.input(ECHO) == 0:
        if time.perf_counter() - start_wait > timeout:
            if debug:
                print("[ERROR] ECHO never went HIGH")
            return None

    pulse_start = time.perf_counter()
    if debug:
        print("[DEBUG] ECHO HIGH detected")

    while GPIO.input(ECHO) == 1:
        if time.perf_counter() - pulse_start > timeout:
            if debug:
                print("[ERROR] ECHO stuck HIGH")
            return None

    pulse_end = time.perf_counter()
    if debug:
        print("[DEBUG] ECHO LOW detected")

    duration = pulse_end - pulse_start
    # Speed of sound = 34300 cm/s; divide by 2 for round-trip.
    distance = (duration * 34300) / 2

    if debug:
        print(f"[DEBUG] Duration: {duration:.6f}s")

    return distance


try:
    print("=== Ultrasonic Sensor Test Start ===\n")

    sample_id = 0
    while True:
        sample_id += 1
        should_print = sample_id % args.print_every == 0
        dist = measure_distance(args.timeout, should_print and not args.quiet_debug)

        if should_print:
            if dist is None:
                print("[WARN] Measurement failed\n")
            else:
                print(f"[INFO] Distance: {dist:.2f} cm\n")

        if args.interval > 0:
            time.sleep(args.interval)

except KeyboardInterrupt:
    print("\nCleaning up...")
finally:
    GPIO.cleanup()
