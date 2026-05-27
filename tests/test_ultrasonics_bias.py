"""Pin down the ultrasonic bias correction so a future edit can't silently
break the calibration.

Calibration source: hal/ultrasonics.py BIAS_SCALE / BIAS_OFFSET
(measured 2026-05-24, FRONT sensor, 2-point fit on medians).
"""

import pytest

from hal.ultrasonics import Ultrasonics


@pytest.mark.parametrize(
    "raw_median_cm,expected_true_cm",
    [
        (9.24, 10.0),   # calibration anchor 1
        (28.71, 30.0),  # calibration anchor 2
    ],
)
def test_bias_correction_matches_calibration_points(
    raw_median_cm: float, expected_true_cm: float
) -> None:
    """The two anchors used to fit must round-trip exactly."""
    assert Ultrasonics._apply_bias(raw_median_cm) == pytest.approx(
        expected_true_cm, abs=0.02
    )


def test_bias_correction_is_monotonic() -> None:
    """Larger raw -> larger corrected, no flipping."""
    values = [5.0, 10.0, 20.0, 50.0, 100.0]
    corrected = [Ultrasonics._apply_bias(v) for v in values]
    assert corrected == sorted(corrected)


def test_bias_correction_makes_short_readings_longer() -> None:
    """Sensor reads short; correction must make the number larger (or equal)."""
    for raw in (5.0, 10.0, 30.0, 100.0):
        assert Ultrasonics._apply_bias(raw) >= raw
