"""Multi-sensor HC-SR04 manager with filtering.

Owns the GPIO setup for the three HC-SR04 sensors and provides one method
``poll(sensor_name)`` that returns the latest filtered distance (or None
on persistent failure).

Filtering:
  - Warmup pings discarded on first use.
  - Sliding window of the last ``window`` raw readings.
  - Filtered value = median (robust to single-ping outliers and timeouts).
  - Failures (None) are kept in the window so persistent failure surfaces
    as a None return.

Why a class:
  The main loop wants a stateful object that carries history per sensor.
  Free functions would force the caller to manage state.

Wiring (BCM): see CLAUDE.md and hardware_check.py. This module relies on
those pin assignments matching reality. ``hardware_check.py`` is the
ground-truth verifier; don't run this in production unless that passes.
"""

from __future__ import annotations

import statistics
import time
from collections import deque
from dataclasses import dataclass, field
from typing import Iterable

# Same pattern as hal/motors.py — keep importable off the Pi for dry-run.
try:
    import RPi.GPIO as GPIO  # type: ignore[import-not-found]
except Exception:  # noqa: BLE001
    GPIO = None  # type: ignore[assignment]

from logs.trace import tracer

ECHO_TIMEOUT_S = 0.03
SOUND_CM_PER_SEC_HALF = 17150.0  # 34300 / 2 (round trip)

# Recommended pin map (CLAUDE.md / hardware_check.py).
DEFAULT_PINS: dict[str, tuple[int, int]] = {
    "front":   (23, 24),
    "left45":  (25, 8),
    "right45": (7, 12),
}

DEFAULT_WINDOW = 5
DEFAULT_WARMUP = 2


@dataclass
class _SensorState:
    name: str
    trig: int
    echo: int
    window: deque[float | None] = field(default_factory=lambda: deque(maxlen=DEFAULT_WINDOW))
    warmed_up: bool = False


class Ultrasonics:
    """Three-HC-SR04 manager with per-sensor median filtering.

    Usage::

        us = Ultrasonics()
        us.setup()
        d = us.poll("front")           # one ping + filter -> cm or None
        d_all = us.poll_all()          # dict[name -> cm or None]
        us.cleanup()
    """

    def __init__(
        self,
        pins: dict[str, tuple[int, int]] | None = None,
        window: int = DEFAULT_WINDOW,
        warmup: int = DEFAULT_WARMUP,
        echo_timeout_s: float = ECHO_TIMEOUT_S,
    ) -> None:
        pins = pins if pins is not None else DEFAULT_PINS
        self._window_size = window
        self._warmup = warmup
        self._timeout = echo_timeout_s
        self._sensors: dict[str, _SensorState] = {
            name: _SensorState(
                name=name,
                trig=trig,
                echo=echo,
                window=deque(maxlen=window),
            )
            for name, (trig, echo) in pins.items()
        }
        self._set_up = False

    # ------------------------------------------------------------------ #
    # Lifecycle
    # ------------------------------------------------------------------ #
    def setup(self) -> None:
        """Configure GPIO pins. Call once before any poll."""
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        for st in self._sensors.values():
            GPIO.setup(st.trig, GPIO.OUT, initial=GPIO.LOW)
            GPIO.setup(st.echo, GPIO.IN)
        time.sleep(0.1)
        self._set_up = True

    def cleanup(self) -> None:
        if self._set_up:
            # Don't call GPIO.cleanup() — other HAL owners may share the chip.
            for st in self._sensors.values():
                try:
                    GPIO.output(st.trig, False)
                except Exception:  # noqa: BLE001 — best-effort cleanup
                    pass
            self._set_up = False

    # ------------------------------------------------------------------ #
    # Polling
    # ------------------------------------------------------------------ #
    def poll(self, name: str) -> float | None:
        """One ping + filter for the given sensor. Returns filtered cm or None."""
        st = self._sensors[name]
        if not st.warmed_up:
            for _ in range(self._warmup):
                self._raw_ping(st)
                time.sleep(0.01)
            st.warmed_up = True

        raw = self._raw_ping(st)
        st.window.append(raw)
        filtered = self._filter(st.window)
        tracer.ultrasonic(
            sensor=name,
            raw_cm=raw,
            filtered_cm=filtered,
            valid=filtered is not None,
        )
        return filtered

    def poll_all(self, names: Iterable[str] | None = None) -> dict[str, float | None]:
        """Round-robin poll. 10ms cooldown between pings keeps echoes clean."""
        names = list(names) if names is not None else list(self._sensors.keys())
        out: dict[str, float | None] = {}
        for n in names:
            out[n] = self.poll(n)
            time.sleep(0.01)
        return out

    # ------------------------------------------------------------------ #
    # Internals
    # ------------------------------------------------------------------ #
    def _raw_ping(self, st: _SensorState) -> float | None:
        GPIO.output(st.trig, False)
        time.sleep(0.002)

        if GPIO.input(st.echo) == 1:
            return None  # echo idle HIGH — wiring problem

        GPIO.output(st.trig, True)
        time.sleep(0.00001)
        GPIO.output(st.trig, False)

        deadline = time.perf_counter() + self._timeout
        while GPIO.input(st.echo) == 0:
            if time.perf_counter() > deadline:
                return None
        pulse_start = time.perf_counter()

        deadline = pulse_start + self._timeout
        while GPIO.input(st.echo) == 1:
            if time.perf_counter() > deadline:
                return None
        pulse_end = time.perf_counter()

        d = (pulse_end - pulse_start) * SOUND_CM_PER_SEC_HALF
        if d < 2 or d > 400:
            return None
        return d

    @staticmethod
    def _filter(window: deque[float | None]) -> float | None:
        good = [v for v in window if v is not None]
        if not good:
            return None
        return statistics.median(good)
