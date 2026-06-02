"""Behavioral tests for ``ultrasonic_direction_check.classify_direction``.

Pins the directional-verification decision logic — the part that catches a
left<->right swap, a dead sensor, or a mis-aimed sensor — without any GPIO.
"""

from __future__ import annotations

from ultrasonic_direction_check import (
    AMBIGUOUS,
    DEAD_EXPECTED,
    NO_DETECTION,
    PASS,
    WRONG_SENSOR,
    classify_direction,
)


# ---------- PASS: each direction lights up its own sensor ---------------


def test_front_object_passes_front() -> None:
    v = classify_direction("front", {"front": 12.0, "left45": 90.0, "right45": 95.0})
    assert v.status == PASS
    assert v.nearest == "front"


def test_left_object_passes_left() -> None:
    v = classify_direction("left", {"front": 80.0, "left45": 13.0, "right45": 92.0})
    assert v.status == PASS
    assert v.nearest == "left45"


def test_right_object_passes_right() -> None:
    v = classify_direction("right", {"front": 85.0, "left45": 88.0, "right45": 11.0})
    assert v.status == PASS
    assert v.nearest == "right45"


# ---------- Swap: the opposite side sees it -> WRONG_SENSOR --------------


def test_left_but_right_sees_it_flags_swap() -> None:
    """Placed near the left, but right45 reads closest -> swap detected."""
    v = classify_direction("left", {"front": 80.0, "left45": 91.0, "right45": 13.0})
    assert v.status == WRONG_SENSOR
    assert v.nearest == "right45"
    assert "바뀐" in v.detail  # swap hint surfaced to the operator


def test_right_but_left_sees_it_flags_swap() -> None:
    v = classify_direction("right", {"front": 85.0, "left45": 12.0, "right45": 89.0})
    assert v.status == WRONG_SENSOR
    assert v.nearest == "left45"
    assert "바뀐" in v.detail


# ---------- Dead expected sensor ----------------------------------------


def test_dead_expected_sensor() -> None:
    v = classify_direction("right", {"front": 85.0, "left45": 88.0, "right45": None})
    assert v.status == DEAD_EXPECTED


def test_dead_expected_even_if_others_alive() -> None:
    """Expected sensor None is DEAD_EXPECTED even when another reads near
    (don't silently 'pass' on a neighbour)."""
    v = classify_direction("left", {"front": 80.0, "left45": None, "right45": 14.0})
    assert v.status == DEAD_EXPECTED


# ---------- Nothing detected by the expected sensor ---------------------


def test_expected_too_far_is_no_detection() -> None:
    """All clear / object not close enough to the expected sensor."""
    v = classify_direction("front", {"front": 70.0, "left45": 85.0, "right45": 90.0})
    assert v.status == NO_DETECTION


# ---------- Ambiguous: expected nearest but a neighbour also close -------


def test_ambiguous_when_two_sensors_see_it() -> None:
    """Expected IS nearest, but another sensor is also under FAR_CM —
    object too central; ask the operator to separate."""
    v = classify_direction("front", {"front": 12.0, "left45": 30.0, "right45": 90.0})
    assert v.status == AMBIGUOUS
    assert v.nearest == "front"


# ---------- Threshold edges ---------------------------------------------


def test_exactly_near_threshold_counts_as_detected() -> None:
    from ultrasonic_direction_check import NEAR_CM

    v = classify_direction(
        "front", {"front": NEAR_CM, "left45": 90.0, "right45": 95.0}
    )
    assert v.status == PASS


# ---------- Regression: expected-near is never a swap (tie/near-tie) -----


def test_expected_near_with_tie_is_not_wrong_sensor() -> None:
    """If the expected sensor itself sees the object near, a TIE with the
    other side must NOT be misread as a left<->right swap. It's AMBIGUOUS at
    worst, never WRONG_SENSOR."""
    v = classify_direction("right", {"front": 80.0, "left45": 20.0, "right45": 20.0})
    assert v.status != WRONG_SENSOR
    assert v.status == AMBIGUOUS


def test_expected_near_even_if_neighbour_marginally_closer_is_not_swap() -> None:
    """Expected (right45) sees 14cm; left45 happens to read 12cm. The right
    sensor DID see the object near, so this is not a swap — AMBIGUOUS."""
    v = classify_direction("right", {"front": 80.0, "left45": 12.0, "right45": 14.0})
    assert v.status != WRONG_SENSOR
    assert v.status == AMBIGUOUS
