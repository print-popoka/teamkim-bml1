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
    APPROACH_SPEED,
    ARC_MIN_CM,
    BASE_SPEED,
    CORNER_ANTICIPATE_CM,
    CORNER_APPROACH_FRACTION,
    CORNER_FLOOR_CM,
    CRUISE,
    CRUISE_REF,
    DEADBAND_CM,
    JUNCTION_CM,
    JUNCTION_COMMIT_CURVATURE,
    JUNCTION_COMMIT_TICKS,
    MAX_DERROR_CM,
    NARROWING_CM,
    SAFE_MARGIN_CM,
    SLOW_FRACTION,
    SLOW_SPEED,
    SPEED_SCALE_FLOOR,
    WallFollowController,
    corner_anticipate_cm,
    front_speed,
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


def test_deadband_zero_curvature_for_tiny_error() -> None:
    """|right - left| below DEADBAND_CM -> exactly zero centering correction.

    Prevents the over-correction loop the professor warned about
    (continuous tiny steering = slow forward progress).
    """
    # Total imbalance < DEADBAND_CM — should be ignored.
    half = DEADBAND_CM / 4  # so total error = DEADBAND_CM/2 < threshold
    cmd = _step(front=80.0, left=15.0 - half, right=15.0 + half)
    assert cmd.action == "arc"
    assert cmd.curvature == 0.0


def test_deadband_lets_real_errors_through() -> None:
    """Errors larger than DEADBAND_CM still produce a correction."""
    cmd = _step(front=80.0, left=10.0, right=10.0 + DEADBAND_CM * 5)
    assert cmd.curvature != 0.0


# ---------- Junction commit -------------------------------------------


def test_junction_right_open_commits_to_right_curve() -> None:
    """Right wall vanishing -> commit to sharp right arc for N ticks."""
    ctrl = WallFollowController()
    # First tick: walls on both sides — normal centering, no commit.
    ctrl.step(80.0, 15.0, 15.0)
    # Second tick: right wall gone. Should trigger commit.
    cmd = ctrl.step(80.0, 15.0, JUNCTION_CM + 10)
    assert cmd.action == "arc"
    assert cmd.curvature == JUNCTION_COMMIT_CURVATURE


def test_junction_commit_holds_through_noisy_readings() -> None:
    """Once committed, the curvature stays locked even if the next tick's
    sensors give different numbers (which they will, mid-rotation)."""
    ctrl = WallFollowController()
    ctrl.step(80.0, 15.0, 15.0)
    ctrl.step(80.0, 15.0, JUNCTION_CM + 10)  # commit fires
    # Now simulate noisy mid-rotation readings.
    cmd = ctrl.step(80.0, 25.0, 35.0)
    assert cmd.curvature == JUNCTION_COMMIT_CURVATURE


def test_junction_commit_releases_after_ticks_expire() -> None:
    """After JUNCTION_COMMIT_TICKS ticks, normal PD takes over."""
    ctrl = WallFollowController()
    ctrl.step(80.0, 15.0, 15.0)
    ctrl.step(80.0, 15.0, JUNCTION_CM + 10)  # commit fires
    for _ in range(JUNCTION_COMMIT_TICKS):
        ctrl.step(80.0, 15.0, 15.0)
    # The next tick should be PD-driven again (zero error => zero curvature)
    cmd = ctrl.step(80.0, 15.0, 15.0)
    assert abs(cmd.curvature) < 0.1


# ---------- D-term clamping -------------------------------------------


def test_derror_clamped_on_big_sensor_jump() -> None:
    """The D-term contribution stays bounded even with huge raw derror.

    Verified via the reason string: the printed ``de`` value should never
    exceed MAX_DERROR_CM in magnitude. The curvature itself may saturate
    (the P term is allowed to do that when the car is genuinely off-center);
    we just want the D term to stop being a separate amplifier.
    """
    ctrl = WallFollowController()
    ctrl.step(80.0, 15.0, 15.0)
    # A 20cm jump in right_cm (under JUNCTION_CM=40 so no commit) —
    # raw derror would be 20 without clamping.
    cmd = ctrl.step(80.0, 15.0, 35.0)
    # Reason carries the *clamped* derror.
    assert "de=" in cmd.reason
    # Pull the de=... number out and check magnitude.
    de_str = cmd.reason.split("de=")[1].split(" ")[0]
    de_val = float(de_str.rstrip("+"))
    assert abs(de_val) <= MAX_DERROR_CM + 1e-6


# ---------- Error-magnitude speed scaling -----------------------------


def test_large_error_reduces_speed_but_not_below_floor() -> None:
    """Big offset -> speed scales down but never below SPEED_SCALE_FLOOR."""
    cmd_big = _step(front=80.0, left=5.0, right=JUNCTION_CM - 2)
    cmd_centered = _step(front=80.0, left=15.0, right=15.0)
    # Bigger error -> slower speed (when not in junction commit).
    # Skip the assertion if junction commit fires (it overrides speed).
    if cmd_big.linear_speed != cmd_centered.linear_speed:
        assert cmd_big.linear_speed < cmd_centered.linear_speed
    assert cmd_big.linear_speed >= BASE_SPEED * SPEED_SCALE_FLOOR - 0.01


# Silence unused-import warnings on constants we expose to other tests.
_ = MAX_DERROR_CM


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


# ---------- Speed table derives from one CRUISE knob (SPEED-2) ----------


def test_speed_table_derives_from_cruise_at_reference() -> None:
    """The whole table is fractions * CRUISE; at the reference cruise it
    reproduces the previously tuned values (45 / ~35 / ~30)."""
    assert BASE_SPEED == CRUISE
    assert APPROACH_SPEED == CRUISE * CORNER_APPROACH_FRACTION
    assert SLOW_SPEED == CRUISE * SLOW_FRACTION
    # Still close to the old hand-tuned magnitudes.
    assert abs(APPROACH_SPEED - 35.0) < 1.0
    assert abs(SLOW_SPEED - 30.0) < 1.0


def test_speed_names_remain_importable() -> None:
    """SLOW_SPEED / APPROACH_SPEED are referenced at runtime by the
    clearance guards + narrowing cap; a missing name is a silent NameError
    on the Pi that pytest would otherwise miss. Pin them positive."""
    assert SLOW_SPEED > 0
    assert APPROACH_SPEED > 0
    assert CRUISE > 0


# ---------- Continuous front-clearance speed profile (SPEED-1) ----------


def test_front_speed_full_cruise_on_open_front() -> None:
    assert front_speed(200.0, CRUISE) == CRUISE


def test_front_speed_floor_is_positive() -> None:
    """Smooth-drive defense: never a zero-speed stall mid-corner."""
    assert front_speed(0.0, CRUISE) > 0.0


def test_front_speed_emits_intermediate_value_on_ramp() -> None:
    """THE discriminator vs the old 3-step block: a strictly-between value
    must exist on the ramp. The step function never produced one."""
    onset = corner_anticipate_cm(CRUISE)
    mid = (CORNER_FLOOR_CM + onset) / 2.0
    floor = CRUISE * CORNER_APPROACH_FRACTION
    s = front_speed(mid, CRUISE)
    assert floor < s < CRUISE


def test_front_speed_monotonic_nondecreasing() -> None:
    """Speed never drops as the corridor opens up. Guards an inverted
    profile. (Supporting check — continuity above is the real proof.)"""
    prev = -1.0
    f = 0.0
    while f <= 200.0:
        s = front_speed(f, CRUISE)
        assert s >= prev - 1e-9
        prev = s
        f += 1.0


def test_front_speed_endpoints_symbolic() -> None:
    """Endpoints checked against the PASSED-IN cruise, not a literal, so
    the contract holds at any hardware-day CRUISE."""
    c = 70.0
    assert front_speed(500.0, c) == c
    assert front_speed(CORNER_FLOOR_CM - 1.0, c) == c * CORNER_APPROACH_FRACTION


def test_step_open_straight_runs_at_cruise() -> None:
    """Centered + far front -> full cruise speed through the controller."""
    cmd = _step(front=200.0, left=15.0, right=15.0)
    assert cmd.action == "arc"
    assert cmd.linear_speed == BASE_SPEED


def test_narrowing_caps_speed_through_step() -> None:
    """Both sides tight (far front) -> speed capped at SLOW_SPEED. Pins the
    re-added CLAUDE.md corridor-width slowdown so it cannot regress."""
    cmd = _step(front=200.0, left=NARROWING_CM - 4, right=NARROWING_CM - 4)
    assert cmd.action == "arc"
    assert cmd.linear_speed <= SLOW_SPEED + 1e-9


# ---------- Speed-aware corner anticipation coupling (SPEED-3) ----------


def test_corner_anticipate_reference_value() -> None:
    """At the reference cruise the onset is exactly the old fixed 25cm, so
    existing corner-anticipation tests stay valid and runtime is unchanged."""
    assert corner_anticipate_cm(CRUISE_REF) == 25.0
    assert CORNER_ANTICIPATE_CM == 25.0


def test_corner_anticipate_couples_to_cruise() -> None:
    """Discriminating sign test on the pure function: a faster cruise begins
    cornering at a strictly LARGER front distance. Passes for any positive
    gain, so the gain magnitude itself stays unguessed."""
    assert corner_anticipate_cm(2 * CRUISE_REF) > corner_anticipate_cm(CRUISE_REF)


def test_ramp_onset_couples_to_cruise() -> None:
    """The speed-ramp onset and the corner-bias onset are the SAME cruise-
    coupled value: speed is full cruise at the onset and below it just
    under, and the onset moves out at higher cruise. One shared invariant."""
    onset_ref = corner_anticipate_cm(CRUISE_REF)
    onset_2x = corner_anticipate_cm(2 * CRUISE_REF)
    assert front_speed(onset_ref, CRUISE_REF) == CRUISE_REF
    assert front_speed(onset_ref - 0.5, CRUISE_REF) < CRUISE_REF
    assert front_speed(onset_2x, 2 * CRUISE_REF) == 2 * CRUISE_REF
    assert front_speed(onset_2x - 0.5, 2 * CRUISE_REF) < 2 * CRUISE_REF
    assert onset_2x > onset_ref
