"""HC-SR04 ultrasonic distance measurement (BML1 lecture).

Wiring (BCM):
    TRIG -> GPIO 23 (output)
    ECHO -> GPIO 24 (input)
    VCC  -> 5V, GND -> GND

Run: python ultrasonic.py     (quit with Ctrl+C)
"""

import time

import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BCM)

TRIG = 23
ECHO = 24

GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)


def measure_distance():
    GPIO.output(TRIG, False)
    time.sleep(0.05)

    # Send 10us trigger pulse.
    GPIO.output(TRIG, True)
    time.sleep(0.00001)
    GPIO.output(TRIG, False)

    print("[DEBUG] TRIG pulse sent")

    timeout = 0.03

    start_wait = time.perf_counter()
    while GPIO.input(ECHO) == 0:
        if time.perf_counter() - start_wait > timeout:
            print("[ERROR] ECHO never went HIGH")
            return None

    pulse_start = time.perf_counter()
    print("[DEBUG] ECHO HIGH detected")

    while GPIO.input(ECHO) == 1:
        if time.perf_counter() - pulse_start > timeout:
            print("[ERROR] ECHO stuck HIGH")
            return None

    pulse_end = time.perf_counter()
    print("[DEBUG] ECHO LOW detected")

    duration = pulse_end - pulse_start
    # Speed of sound = 34300 cm/s; divide by 2 for round-trip.
    distance = (duration * 34300) / 2

    print(f"[DEBUG] Duration: {duration:.6f}s")

    return distance


try:
    print("=== Ultrasonic Sensor Test Start ===\n")

    while True:
        dist = measure_distance()

        if dist is None:
            print("[WARN] Measurement failed\n")
        else:
            print(f"[INFO] Distance: {dist:.2f} cm\n")

        time.sleep(1)

except KeyboardInterrupt:
    print("\nCleaning up...")
    GPIO.cleanup()
