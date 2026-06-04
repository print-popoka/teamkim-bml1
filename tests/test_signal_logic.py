"""Tests for the RED-priority traffic-light decision rule (PERC-1).

These run on Mac because the logic lives in the cv2-free
``perception.signal_logic`` module (the cv2 detector is not importable here).
"""

from __future__ import annotations

from perception.signal_logic import MIN_AREA, decide_signal


# ---- RED priority: a present RED must STOP regardless of green ----------


def test_red_present_stops_even_with_larger_green() -> None:
    """THE PERC-1 fix: the old symmetric area-compare returned GO here
    (green >= red*1.5); RED-priority now returns STOP."""
    assert decide_signal(red_area=4000, green_area=6000) == "STOP"


def test_red_present_stops_with_comparable_green() -> None:
    assert decide_signal(4000, 5000) == "STOP"


def test_red_present_stops_with_equal_green() -> None:
    assert decide_signal(4000, 4000) == "STOP"


def test_substantial_red_alone_stops() -> None:
    assert decide_signal(4000, 0) == "STOP"


# ---- GREEN only / nothing present ---------------------------------------


def test_green_only_goes() -> None:
    assert decide_signal(0, 4000) == "GO"


def test_nothing_present_is_unknown() -> None:
    assert decide_signal(0, 0) == "UNKNOWN"


def test_both_below_min_area_is_unknown() -> None:
    assert decide_signal(MIN_AREA, MIN_AREA) == "UNKNOWN"


# ---- GREEN stays conservative when no decision-critical red -------------


def test_marginal_green_without_margin_is_unknown() -> None:
    """red just under MIN_AREA, green only marginally larger -> not a clear
    GO (win-margin keeps GREEN conservative so it can't release a real RED)."""
    assert decide_signal(190, 210) == "UNKNOWN"


def test_clear_green_with_negligible_red_goes() -> None:
    assert decide_signal(10, 4000) == "GO"
