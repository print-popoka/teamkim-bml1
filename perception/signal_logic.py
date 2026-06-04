"""Pure, cv2-free traffic-light decision logic.

Extracted from ``traffic_light.py`` so the RED/GREEN decision rule is
importable and unit-testable on a dev machine without OpenCV. The detector
in ``traffic_light.py`` imports ``decide_signal`` from here, so this module
is the single source of truth for the decision rule.

Safety semantics (CLAUDE.md):
  - RED is the only decision-critical color and MUST never be masked by a
    co-occurring GREEN — see ``decide_signal``.
  - GREEN == UNKNOWN == "keep doing what you were doing" in the moving
    states; only RED forces a stop. While STOPPED_AT_RED, the state machine
    additionally requires an explicit GREEN to release.
"""

from __future__ import annotations

from typing import Literal

Signal = Literal["STOP", "GO", "UNKNOWN"]

# Minimum circular area (px) for a color blob to count as a real signal.
MIN_AREA = 200
# A winner must beat the other color by this factor to commit (keeps GREEN
# conservative so a faint/partial green can't be called GO).
WIN_MARGIN = 1.5


def decide_signal(red_area: float, green_area: float) -> Signal:
    """Decide STOP / GO / UNKNOWN from the two circular color areas.

    RED-PRIORITY: a substantial RED present -> STOP, regardless of any
    co-occurring (even larger) GREEN. The previous symmetric area-compare
    let a larger/closer GREEN mask a present RED into GO/UNKNOWN, which
    violates the locked "RED must never be missed" rule; a spurious stop is
    the safe failure direction.

    When no decision-critical RED is present, GREEN stays conservative (must
    clear MIN_AREA *and* the win-margin) so a false GREEN can't release a
    real RED stop.
    """
    if red_area > MIN_AREA:
        return "STOP"
    if green_area > MIN_AREA and green_area >= red_area * WIN_MARGIN:
        return "GO"
    return "UNKNOWN"
