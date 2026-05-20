"""L298N 2-channel motor driver, smooth-drive primitive.

Per PI feedback (CLAUDE.md: "Driving style — smooth, never stop-and-turn"),
the canonical motor primitive is ``drive(left_pwm, right_pwm)`` where each
PWM is a SIGNED value in [-100, +100]:

  +50 = that side forward at 50% duty
   0  = that side coasting (both IN pins low, no electrical brake)
  -50 = that side reverse at 50% duty

Higher-level helpers (forward, arc, pivot, stop) wrap drive().

Calibration constants are PLACEHOLDERS until the sample-maze test fills
them in via ``motor/motor_calibration.py``. The summary printout there
maps 1-to-1 onto these names.

Trace integration: every state-changing call emits one tracer.motor()
event so reaction-delay can be computed offline.
"""

from __future__ import annotations

from dataclasses import dataclass

# RPi.GPIO is Pi-only. Off-Pi we still want this module importable so
# `python main.py --dry-run` works on a dev machine. Hardware paths
# check ``GPIO is not None`` (effectively via ``self._dry_run``).
try:
    import RPi.GPIO as GPIO  # type: ignore[import-not-found]
except Exception:  # noqa: BLE001
    GPIO = None  # type: ignore[assignment]

from logs.trace import tracer

# ---------------------------------------------------------------------- #
# Pin map — must match motor/motor.py, motor/motor_calibration.py,
#           hardware_check.py.
# ---------------------------------------------------------------------- #
LEFT_IN1, LEFT_IN2 = 17, 27
RIGHT_IN3, RIGHT_IN4 = 22, 5
LEFT_ENA, RIGHT_ENB = 18, 19

PWM_FREQ = 1000

# ---------------------------------------------------------------------- #
# Calibration constants — PLACEHOLDERS. Replace with measured values from
# motor/motor_calibration.py once sample-maze test data is in.
# ---------------------------------------------------------------------- #
MIN_PWM_LEFT: int = 20
MIN_PWM_RIGHT: int = 20
MAX_PWM: int = 90

LEFT_TRIM: float = 1.0
RIGHT_TRIM: float = 1.0

SPEED_CM_PER_SEC_AT_50: float = 25.0   # placeholder
TURN_DEG_PER_SEC_AT_50: float = 150.0  # placeholder


@dataclass(frozen=True)
class MotorCommand:
    left_pwm: float
    right_pwm: float
    description: str


class Motors:
    """L298N two-channel driver. Owns IN1..4 + ENA/ENB + PWMs."""

    def __init__(self, dry_run: bool = False) -> None:
        self._dry_run = dry_run
        self._pwm_a = None
        self._pwm_b = None
        self._set_up = False
        self.last: MotorCommand = MotorCommand(0.0, 0.0, "init")

    # ------------------------------------------------------------------ #
    # Lifecycle
    # ------------------------------------------------------------------ #
    def setup(self) -> None:
        if self._dry_run:
            self._set_up = True
            return
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        for p in (LEFT_IN1, LEFT_IN2, RIGHT_IN3, RIGHT_IN4, LEFT_ENA, RIGHT_ENB):
            GPIO.setup(p, GPIO.OUT, initial=GPIO.LOW)
        self._pwm_a = GPIO.PWM(LEFT_ENA, PWM_FREQ)
        self._pwm_b = GPIO.PWM(RIGHT_ENB, PWM_FREQ)
        self._pwm_a.start(0)
        self._pwm_b.start(0)
        self._set_up = True

    def cleanup(self) -> None:
        try:
            self.stop()
        finally:
            if not self._dry_run and self._set_up:
                if self._pwm_a is not None:
                    self._pwm_a.stop()
                if self._pwm_b is not None:
                    self._pwm_b.stop()
                for p in (LEFT_IN1, LEFT_IN2, RIGHT_IN3, RIGHT_IN4):
                    try:
                        GPIO.output(p, False)
                    except Exception:  # noqa: BLE001
                        pass
            self._set_up = False

    # ------------------------------------------------------------------ #
    # Primary primitive
    # ------------------------------------------------------------------ #
    def drive(
        self,
        left_pwm: float,
        right_pwm: float,
        *,
        description: str = "drive",
    ) -> None:
        """Apply signed PWMs to each side. -100..+100. 0 = coast."""
        l = self._apply_deadband(self._clamp(left_pwm * LEFT_TRIM), MIN_PWM_LEFT)
        r = self._apply_deadband(self._clamp(right_pwm * RIGHT_TRIM), MIN_PWM_RIGHT)

        if not self._dry_run and self._set_up:
            self._set_side(LEFT_IN1, LEFT_IN2, l)
            self._set_side(RIGHT_IN3, RIGHT_IN4, r)
            assert self._pwm_a is not None and self._pwm_b is not None
            self._pwm_a.ChangeDutyCycle(abs(l))
            self._pwm_b.ChangeDutyCycle(abs(r))

        self.last = MotorCommand(l, r, description)
        tracer.motor(left_pwm=l, right_pwm=r, direction=description)

    # ------------------------------------------------------------------ #
    # High-level helpers
    # ------------------------------------------------------------------ #
    def forward(self, speed: float = 50.0) -> None:
        self.drive(speed, speed, description=f"forward({speed:.0f})")

    def backward(self, speed: float = 50.0) -> None:
        self.drive(-speed, -speed, description=f"backward({speed:.0f})")

    def stop(self) -> None:
        self.drive(0, 0, description="stop")

    def arc(self, speed: float, curvature: float) -> None:
        """Continuous smooth arc.

        curvature in [-1, +1]: -1 = sharp right, 0 = straight, +1 = sharp left.
        The inside wheel is slowed; the outside wheel holds requested speed.
        Both wheels remain forward — no in-place pivot.
        """
        c = max(-1.0, min(1.0, curvature))
        if c >= 0:
            left = speed * (1.0 - c)
            right = speed
        else:
            right = speed * (1.0 + c)
            left = speed
        self.drive(left, right, description=f"arc({speed:.0f},{c:+.2f})")

    def pivot_right(self, speed: float = 40.0) -> None:
        """In-place right pivot. FALLBACK ONLY — dead-end U-turns."""
        self.drive(speed, -speed, description=f"pivot_right({speed:.0f})")

    def pivot_left(self, speed: float = 40.0) -> None:
        """In-place left pivot. FALLBACK ONLY — dead-end U-turns."""
        self.drive(-speed, speed, description=f"pivot_left({speed:.0f})")

    # ------------------------------------------------------------------ #
    # Internals
    # ------------------------------------------------------------------ #
    @staticmethod
    def _clamp(v: float) -> float:
        return max(-float(MAX_PWM), min(float(MAX_PWM), v))

    @staticmethod
    def _apply_deadband(v: float, floor: int) -> float:
        if v == 0:
            return 0.0
        sign = 1 if v > 0 else -1
        return float(sign * max(floor, abs(v)))

    @staticmethod
    def _set_side(in_a: int, in_b: int, signed_pwm: float) -> None:
        if signed_pwm > 0:
            GPIO.output(in_a, True)
            GPIO.output(in_b, False)
        elif signed_pwm < 0:
            GPIO.output(in_a, False)
            GPIO.output(in_b, True)
        else:
            GPIO.output(in_a, False)
            GPIO.output(in_b, False)
