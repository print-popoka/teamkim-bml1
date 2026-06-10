"""Motor + ultrasonic assembly check for the BML1 robot.

Run this on the Raspberry Pi after lifting the wheels off the ground:
    python hardware_check.py

Lecture wiring reference (BCM):
    Motor A: IN1=17, IN2=27, ENA=18
    Motor B: IN3=22, IN4=5,  ENB=19
    Ultrasonic FRONT: TRIG=23, ECHO=24

Project wiring extends the lecture setup to three HC-SR04 sensors:
    LEFT45:  TRIG=25, ECHO=8
    RIGHT45: TRIG=7,  ECHO=12
"""

from __future__ import annotations

import argparse
import statistics
import sys
import time
from dataclasses import dataclass

try:
    import RPi.GPIO as GPIO
except ImportError:
    GPIO = None


PWM_FREQ = 1000
SOUND_SPEED_CM_PER_SEC = 34300

MOTOR_PINS = {
    "left": {"in1": 22, "in2": 5, "enable": 19},
    "right": {"in1": 17, "in2": 27, "enable": 18},
}


@dataclass(frozen=True)
class UltrasonicSensor:
    name: str
    trig: int
    echo: int


ULTRASONIC_SENSORS = [
    UltrasonicSensor("front", 23, 24),
    UltrasonicSensor("left45", 25, 8),
    UltrasonicSensor("right45", 7, 12),
]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Check L298N motor channels and HC-SR04 ultrasonic sensors."
    )
    parser.add_argument(
        "--front-only",
        action="store_true",
        help="test only the lecture front ultrasonic pair, GPIO 23/24",
    )
    parser.add_argument(
        "--skip-motor",
        action="store_true",
        help="skip the motor movement test",
    )
    parser.add_argument(
        "--skip-ultrasonic",
        action="store_true",
        help="skip ultrasonic measurement tests",
    )
    parser.add_argument(
        "--samples",
        type=positive_int,
        default=5,
        help="ultrasonic samples per sensor",
    )
    parser.add_argument(
        "--warmup",
        type=non_negative_int,
        default=1,
        help="discard this many initial pings per sensor (HC-SR04 first ping is noisy)",
    )
    parser.add_argument(
        "--motor-speed",
        type=duty_cycle,
        default=25,
        help="motor PWM duty cycle, 0-100",
    )
    parser.add_argument(
        "--motor-duration",
        type=non_negative_float,
        default=0.7,
        help="seconds for each motor pulse",
    )
    parser.add_argument(
        "--ultrasonic-timeout",
        type=non_negative_float,
        default=0.015,
        help="seconds to wait for each HC-SR04 echo edge",
    )
    parser.add_argument(
        "--yes",
        action="store_true",
        help="do not pause before motor movement",
    )
    return parser.parse_args()


def positive_int(raw: str) -> int:
    value = int(raw)
    if value < 1:
        raise argparse.ArgumentTypeError("must be >= 1")
    return value


def non_negative_int(raw: str) -> int:
    value = int(raw)
    if value < 0:
        raise argparse.ArgumentTypeError("must be >= 0")
    return value


def non_negative_float(raw: str) -> float:
    value = float(raw)
    if value < 0:
        raise argparse.ArgumentTypeError("must be >= 0")
    return value


def duty_cycle(raw: str) -> int:
    value = int(raw)
    if not 0 <= value <= 100:
        raise argparse.ArgumentTypeError("must be between 0 and 100")
    return value


def require_gpio() -> None:
    if GPIO is None:
        print("[FAIL] RPi.GPIO is not installed. Run this script on the Raspberry Pi.")
        sys.exit(1)


def print_wiring_reference(sensors: list[UltrasonicSensor]) -> None:
    print("=== Wiring reference, BCM numbering ===")
    print("Motor A/left:  IN1=17 IN2=27 ENA=18")
    print("Motor B/right: IN3=22 IN4=5  ENB=19")
    for sensor in sensors:
        print(f"Ultrasonic {sensor.name}: TRIG={sensor.trig} ECHO={sensor.echo}")
    print("Power: L298N motor supply=12V, Raspberry Pi GND and driver GND common")
    print("HC-SR04: VCC=5V, GND=GND, Echo is 5V; voltage divider is recommended")
    print()


def setup_gpio(sensors: list[UltrasonicSensor], include_motor: bool) -> None:
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)

    if include_motor:
        for pins in MOTOR_PINS.values():
            GPIO.setup(pins["in1"], GPIO.OUT, initial=GPIO.LOW)
            GPIO.setup(pins["in2"], GPIO.OUT, initial=GPIO.LOW)
            GPIO.setup(pins["enable"], GPIO.OUT, initial=GPIO.LOW)

    for sensor in sensors:
        GPIO.setup(sensor.trig, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(sensor.echo, GPIO.IN)

    time.sleep(0.1)


def clamp_speed(speed: int) -> int:
    return max(0, min(100, speed))


def set_motor_channel(pins: dict[str, int], pwm, direction: int, speed: int) -> None:
    if direction > 0:
        GPIO.output(pins["in1"], GPIO.HIGH)
        GPIO.output(pins["in2"], GPIO.LOW)
    elif direction < 0:
        GPIO.output(pins["in1"], GPIO.LOW)
        GPIO.output(pins["in2"], GPIO.HIGH)
    else:
        GPIO.output(pins["in1"], GPIO.LOW)
        GPIO.output(pins["in2"], GPIO.LOW)
    pwm.ChangeDutyCycle(clamp_speed(speed) if direction else 0)


def stop_motors(pwms: dict[str, object]) -> None:
    for side, pins in MOTOR_PINS.items():
        set_motor_channel(pins, pwms[side], 0, 0)


def run_motor_test(speed: int, duration: float, skip_prompt: bool) -> None:
    print("=== Motor test ===")
    print("Lift the wheels off the ground before continuing.")
    print("Expected order: left forward, left backward, right forward, right backward, both forward.")
    if not skip_prompt:
        input("Press Enter to start the motor pulses, or Ctrl+C to cancel...")

    pwms = {
        side: GPIO.PWM(pins["enable"], PWM_FREQ)
        for side, pins in MOTOR_PINS.items()
    }
    for pwm in pwms.values():
        pwm.start(0)

    steps = [
        ("left forward", {"left": 1, "right": 0}),
        ("left backward", {"left": -1, "right": 0}),
        ("right forward", {"left": 0, "right": 1}),
        ("right backward", {"left": 0, "right": -1}),
        ("both forward", {"left": 1, "right": 1}),
    ]

    try:
        for label, directions in steps:
            print(f"[CHECK] {label} at {clamp_speed(speed)}% PWM")
            for side, direction in directions.items():
                set_motor_channel(MOTOR_PINS[side], pwms[side], direction, speed)
            time.sleep(duration)
            stop_motors(pwms)
            time.sleep(0.4)
        print("[PASS?] Motor GPIO/PWM commands completed. Confirm wheel direction by sight.")
        print("        If one side spins backward, swap that channel's IN1/IN2 or motor output wires.")
    finally:
        stop_motors(pwms)
        for pwm in pwms.values():
            pwm.stop()
    print()


def wait_for_echo_state(pin: int, target: int, timeout: float) -> float | None:
    start = time.perf_counter()
    while GPIO.input(pin) != target:
        now = time.perf_counter()
        if now - start > timeout:
            return None
    return time.perf_counter()


def measure_distance(sensor: UltrasonicSensor, timeout: float = 0.015) -> tuple[float | None, str]:
    GPIO.output(sensor.trig, GPIO.LOW)
    # Longer settle reduces the first-ping outlier (HC-SR04 needs the
    # transmitter capacitor to bleed off between pulses).
    time.sleep(0.002)

    if GPIO.input(sensor.echo) == GPIO.HIGH:
        return None, "echo idle HIGH; check ECHO wiring/GND"

    GPIO.output(sensor.trig, GPIO.HIGH)
    time.sleep(0.00001)
    GPIO.output(sensor.trig, GPIO.LOW)

    pulse_start = wait_for_echo_state(sensor.echo, GPIO.HIGH, timeout)
    if pulse_start is None:
        return None, "echo never went HIGH; check TRIG/ECHO/VCC/GND"

    pulse_end = wait_for_echo_state(sensor.echo, GPIO.LOW, timeout)
    if pulse_end is None:
        return None, "echo stayed HIGH; object may be too far or ECHO wiring is wrong"

    duration = pulse_end - pulse_start
    distance = (duration * SOUND_SPEED_CM_PER_SEC) / 2
    if not 2 <= distance <= 400:
        return distance, "out of HC-SR04 nominal range"
    return distance, "ok"


def run_ultrasonic_test(
    sensors: list[UltrasonicSensor],
    samples: int,
    timeout: float,
    warmup: int = 1,
) -> None:
    print("=== Ultrasonic test ===")
    print("Put a flat object 20-50 cm in front of each sensor while it is tested.")
    overall_pass = True

    for sensor in sensors:
        print(f"[CHECK] {sensor.name} TRIG={sensor.trig} ECHO={sensor.echo}")
        # Warmup pings — discarded. HC-SR04 first ping after idle is unreliable.
        for _ in range(max(0, warmup)):
            measure_distance(sensor, timeout)
            time.sleep(0.06)
        distances = []
        failures = []
        for idx in range(max(1, samples)):
            distance, status = measure_distance(sensor, timeout)
            if distance is None:
                failures.append(status)
                print(f"  sample {idx + 1}: FAIL - {status}")
            else:
                distances.append(distance)
                print(f"  sample {idx + 1}: {distance:6.2f} cm - {status}")
            time.sleep(0.06)

        if distances:
            median = statistics.median(distances)
            spread = max(distances) - min(distances)
            stable = spread <= 10 or len(distances) == 1
            status = "PASS" if not failures and stable else "WARN"
            overall_pass = overall_pass and status == "PASS"
            print(f"  result: {status} median={median:.2f}cm spread={spread:.2f}cm")
            if not stable:
                print("          readings are noisy; check sensor angle, loose wires, and target surface")
        else:
            overall_pass = False
            unique_failures = sorted(set(failures))
            print(f"  result: FAIL ({'; '.join(unique_failures)})")
        print()

    if overall_pass:
        print("[PASS] Ultrasonic GPIO transitions and distance readings look usable.")
    else:
        print("[WARN] One or more ultrasonic checks need wiring/placement review.")
    print()


def main() -> None:
    args = parse_args()
    require_gpio()

    sensors = ULTRASONIC_SENSORS[:1] if args.front_only else ULTRASONIC_SENSORS
    include_motor = not args.skip_motor

    print_wiring_reference(sensors)
    setup_gpio(sensors, include_motor)

    try:
        if not args.skip_ultrasonic:
            run_ultrasonic_test(
                sensors,
                args.samples,
                args.ultrasonic_timeout,
                args.warmup,
            )
        if include_motor:
            run_motor_test(args.motor_speed, args.motor_duration, args.yes)
    finally:
        GPIO.cleanup()
        print("GPIO cleanup complete.")


if __name__ == "__main__":
    main()
