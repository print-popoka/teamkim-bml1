"""Traffic light detection for the main control loop.

Wraps the proven HSV + circularity logic from ``camera/hsv_circle.py``
into a reusable class. The standalone scripts in ``camera/`` stay around
as the calibration/debug entry points; this is what the runtime calls.

Per CLAUDE.md:
  - RED -> STOP, GREEN -> GO, anything else -> UNKNOWN (no yellow).
  - Safety semantics: GREEN and UNKNOWN both mean "continue what you were
    doing"; only RED is decision-critical. (Main-loop state machine adds
    the "stopped at red waits for explicit GREEN" rule.)
  - HSV thresholds are conservative-on-GREEN; tuned against printed-paper
    samples; real LED traffic lights will produce stronger signals and
    will easily pass.
  - 5-frame majority-vote temporal smoothing.
"""

from __future__ import annotations

from collections import Counter, deque
from dataclasses import dataclass
from typing import Literal

import cv2
import numpy as np

from logs.trace import tracer

Signal = Literal["STOP", "GO", "UNKNOWN"]


# HSV thresholds — single source of truth. The debug CLI
# camera/hsv_circle.py imports these (with snake_case aliases) so the
# two cannot drift. Update here when re-calibrating against new lighting.
RED_LOWER_1 = np.array([0, 150, 100])
RED_UPPER_1 = np.array([12, 255, 255])
RED_LOWER_2 = np.array([165, 150, 100])
RED_UPPER_2 = np.array([179, 255, 255])
GREEN_LOWER = np.array([35, 135, 100])
GREEN_UPPER = np.array([90, 255, 255])

MIN_AREA = 200
MIN_CIRCULARITY = 0.55
MIN_RADIUS = 6
MAX_RADIUS = 140

WIN_MARGIN = 1.5

SMOOTH_WINDOW = 5
SMOOTH_MIN_VOTES = 3

_KERNEL = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))


@dataclass
class DetectorReading:
    signal: Signal
    raw_signal: Signal
    red_area: int
    green_area: int
    frame_id: int | None = None


class TrafficLightDetector:
    """Stateful detector. Call ``detect(frame)`` per frame.

    The frame must be a numpy ndarray as returned by picamera2 with format
    "RGB888" — which, per libcamera quirk, is actually BGR-ordered. The
    detector therefore uses ``cv2.COLOR_BGR2HSV``.
    """

    def __init__(
        self,
        roi_height_ratio: float = 0.7,
        smooth_window: int = SMOOTH_WINDOW,
        smooth_min_votes: int = SMOOTH_MIN_VOTES,
    ) -> None:
        self._roi_ratio = roi_height_ratio
        self._history: deque[Signal] = deque(maxlen=smooth_window)
        self._smooth_min_votes = smooth_min_votes

    def detect(self, frame: np.ndarray, frame_id: int | None = None) -> DetectorReading:
        h = frame.shape[0]
        roi = frame[: int(h * self._roi_ratio), :]

        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        red_mask_1 = cv2.inRange(hsv, RED_LOWER_1, RED_UPPER_1)
        red_mask_2 = cv2.inRange(hsv, RED_LOWER_2, RED_UPPER_2)
        red_mask = _clean(cv2.bitwise_or(red_mask_1, red_mask_2))
        green_mask = _clean(cv2.inRange(hsv, GREEN_LOWER, GREEN_UPPER))

        red_area = _circular_area(red_mask)
        green_area = _circular_area(green_mask)

        raw = _decide(red_area, green_area)
        smoothed = self._smooth(raw)

        tracer.camera(
            signal=smoothed,
            red_area=red_area,
            green_area=green_area,
            frame_id=frame_id,
        )
        return DetectorReading(
            signal=smoothed,
            raw_signal=raw,
            red_area=red_area,
            green_area=green_area,
            frame_id=frame_id,
        )

    # ------------------------------------------------------------------ #
    # Internals
    # ------------------------------------------------------------------ #
    def _smooth(self, raw: Signal) -> Signal:
        self._history.append(raw)
        if self._history.maxlen is None or len(self._history) < self._history.maxlen:
            return raw
        top, votes = Counter(self._history).most_common(1)[0]
        if votes >= self._smooth_min_votes:
            return top  # type: ignore[return-value]
        return "UNKNOWN"


def _clean(mask: np.ndarray) -> np.ndarray:
    m = cv2.morphologyEx(mask, cv2.MORPH_OPEN, _KERNEL, iterations=1)
    m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, _KERNEL, iterations=2)
    return m


def _circular_area(mask: np.ndarray) -> int:
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    total = 0
    for contour in contours:
        area = cv2.contourArea(contour)
        if area < MIN_AREA:
            continue
        perim = cv2.arcLength(contour, True)
        if perim == 0:
            continue
        circularity = 4 * np.pi * area / (perim * perim)
        (_x, _y), radius = cv2.minEnclosingCircle(contour)
        if circularity >= MIN_CIRCULARITY and MIN_RADIUS <= radius <= MAX_RADIUS:
            total += int(area)
    return total


def _decide(red_area: int, green_area: int) -> Signal:
    if red_area <= MIN_AREA and green_area <= MIN_AREA:
        return "UNKNOWN"
    if red_area > green_area:
        return "STOP" if red_area >= green_area * WIN_MARGIN else "UNKNOWN"
    return "GO" if green_area >= red_area * WIN_MARGIN else "UNKNOWN"
