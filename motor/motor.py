"""L298N two-motor control via RPi.GPIO + PWM (BML1 Motor lecture).

Wiring (BCM):
    IN1 -> GPIO 17, IN2 -> GPIO 27   (Motor A, Left)
    IN3 -> GPIO 22, IN4 -> GPIO 5    (Motor B, Right)
    ENA -> GPIO 18, ENB -> GPIO 19   (PWM speed)

Run: python motor.py
"""

import time

import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

IN1, IN2, IN3, IN4 = 17, 27, 22, 5
ENA, ENB = 18, 19

for p in [IN1, IN2, IN3, IN4, ENA, ENB]:
    GPIO.setup(p, GPIO.OUT)

PWM_FREQ = 1000

pwmA = GPIO.PWM(ENA, PWM_FREQ)
pwmB = GPIO.PWM(ENB, PWM_FREQ)

pwmA.start(0)
pwmB.start(0)


def set_speed(speed):
    speed = max(0, min(100, speed))
    pwmA.ChangeDutyCycle(speed)
    pwmB.ChangeDutyCycle(speed)


def forward(speed):
    GPIO.output(IN1, True)
    GPIO.output(IN2, False)
    GPIO.output(IN3, True)
    GPIO.output(IN4, False)
    set_speed(speed)


def stop():
    set_speed(0)
    for p in [IN1, IN2, IN3, IN4]:
        GPIO.output(p, False)


try:
    for SPEED in [20, 40]:
        print(f"Forward SPEED={SPEED}")
        forward(SPEED)
        time.sleep(2)

    print("Stop")
    stop()
    time.sleep(1)

finally:
    stop()
    pwmA.stop()
    pwmB.stop()
    GPIO.cleanup()
