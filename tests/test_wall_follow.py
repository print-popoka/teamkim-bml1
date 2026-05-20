"""Behavioral tests for ``control.wall_follow.WallFollowController``.

These pin down the *intent* of each branch, not the exact numerical
output. When we retune placeholder constants after the sample-maze
test, the assertions about *direction* and *which branch fires* should
keep holding.

Curvature sign convention:
    +1.0 = sharp LEFT, 0 = straight, -1.0 = sharp RIGHT
"""

from __future__ import annotations

from control.wall_follow import (
    ARC_MIN_CM,
    CORNER_ANTICIPATE_CM,
    SAFE_MARGIN_CM,
    WallFollowController,
)


def _step(front, left, right):
    ctrl = WallFollowController()
    return ctrl.step(front, left, right)


# ---------- Clearance guards -------------------------------------------


def test_clearance_guard_left_curves_right() -> None:
    """Too close on left -> sharp right curve, slowed."""
    cmd = _step(front=80.0, left=SAFE_MARGIN_CM - 1, right=20.0)
    assert cmd.action == "arc"
    assert cmd.curvature < 0, "should curve right (negative curvature)"
    assert cmd.linear_speed > 0


def test_clearance_guard_right_curves_left() -> None:
    cmd = _step(front=80.0, left=20.0, right=SAFE_MARGIN_CM - 1)
    assert cmd.action == "arc"
    assert cmd.curvature > 0


# ---------- Pivot fallback ---------------------------------------------


def test_pivot_when_all_sides_too_close() -> None:
    """Dead-end: front + both sides under ARC_MIN -> pivot."""
    cmd = _step(
        front=ARC_MIN_CM - 1,
        left=ARC_MIN_CM - 1,
        right=ARC_MIN_CM - 1,
    )
    assert cmd.action.startswith("pivot")


def test_no_pivot_when_one_side_clear() -> None:
    """One side has room -> arc (smooth), not pivot."""
    cmd = _step(front=ARC_MIN_CM - 1, left=20.0, right=ARC_MIN_CM - 1)
    assert cmd.action == "arc"


# ---------- Centering PD -----------------------------------------------


def test_centering_more_room_right_curves_right() -> None:
    """error = right - left > 0 -> curve right (negative curvature)."""
    cmd = _step(front=80.0, left=10.0, right=20.0)
    assert cmd.action == "arc"
    assert cmd.curvature < 0


def test_centering_more_room_left_curves_left() -> None:
    cmd = _step(front=80.0, left=20.0, right=10.0)
    assert cmd.action == "arc"
    assert cmd.curvature > 0


def test_centering_balanced_goes_straight_or_nearly() -> None:
    """Equal side distances + far front -> near-zero curvature."""
    cmd = _step(front=80.0, left=15.0, right=15.0)
    assert cmd.action == "arc"
    assert abs(cmd.curvature) < 0.1


# ---------- Corner anticipation ----------------------------------------


def test_corner_anticipation_kicks_in_with_close_front() -> None:
    """Front shortening while centered -> non-zero curvature toward open side."""
    far_cmd = _step(front=80.0, left=15.0, right=15.0)
    near_cmd = _step(front=CORNER_ANTICIPATE_CM - 5, left=15.0, right=15.0)
    assert abs(near_cmd.curvature) > abs(far_cmd.curvature)


def test_corner_anticipation_chooses_more_open_side() -> None:
    cmd_more_right = _step(front=CORNER_ANTICIPATE_CM - 5, left=10.0, right=25.0)
    cmd_more_left = _step(front=CORNER_ANTICIPATE_CM - 5, left=25.0, right=10.0)
    assert cmd_more_right.curvature < 0  # curve right
    assert cmd_more_left.curvature > 0


# ---------- None inputs ------------------------------------------------


def test_none_inputs_default_to_far() -> None:
    """All None -> open corridor -> arc with near-zero curvature."""
    cmd = _step(front=None, left=None, right=None)
    assert cmd.action == "arc"
    assert abs(cmd.curvature) < 0.1
