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
    ACCEL_PWM_PER_TICK,
    APPROACH_SPEED,
    ARC_MIN_CM,
    BASE_SPEED,
    CENTERING_MAX,
    CORNER_ANTICIPATE_CM,
    CORNER_ANTICIPATE_REF_CM,
    CORNER_APPROACH_FRACTION,
    CORNER_FLOOR_CM,
    CRUISE,
    CRUISE_REF,
    DEAD_SENSOR_TICKS,
    DEADBAND_CM,
    FRONT_NONE_HOLD_TICKS,
    FRONT_PIVOT_CM,
    MAX_CURVATURE_STEP,
    MAX_DERROR_CM,
    NARROWING_CM,
    REACQ_CURVATURE,
    REACQ_FRONT_ABORT_CM,
    REACQ_MAX_TICKS,
    REACQ_NONE_TICKS,
    REACQ_OPEN_CM,
    REACQ_SPEED,
    REACQ_STRAIGHT_TICKS,
    SAFE_MARGIN_CM,
    SLOW_FRACTION,
    SLOW_SPEED,
    SPEED_SCALE_FLOOR,
    WIDE_AREA_SPEED_FRACTION,
    WIDE_OPEN_CM,
    WallFollowController,
    corner_anticipate_cm,
    front_speed,
)


def _step(front, left, right):
    ctrl = WallFollowController()
    return ctrl.step(front, left, right)


def _armed_controller(right: float = 20.0) -> WallFollowController:
    """Controller that has followed a right wall long enough to arm the
    wall-reacquire turn."""
    ctrl = WallFollowController()
    for _ in range(3):
        ctrl.step(80.0, 15.0, right)
    return ctrl


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


# ---------- Pivot fallbacks ---------------------------------------------


def test_pivot_when_all_sides_too_close() -> None:
    """Dead-end: front + both sides under ARC_MIN -> pivot."""
    cmd = _step(
        front=ARC_MIN_CM - 1,
        left=ARC_MIN_CM - 1,
        right=ARC_MIN_CM - 1,
    )
    assert cmd.action.startswith("pivot")


def test_no_pivot_when_one_side_clear() -> None:
    """One side has room (and front above the emergency line) -> arc."""
    cmd = _step(front=ARC_MIN_CM - 1, left=20.0, right=ARC_MIN_CM - 1)
    assert cmd.action == "arc"


def test_dead_end_pivot_prefers_open_side_tie_right() -> None:
    """Symmetric dead-end keeps the right-hand convention."""
    cmd = _step(front=5.0, left=5.0, right=5.0)
    assert cmd.action == "pivot_right"


def test_dead_end_pivot_chooses_more_open_side() -> None:
    cmd = _step(front=5.0, left=11.0, right=5.0)
    assert cmd.action == "pivot_left"


def test_front_emergency_pivots_toward_open_side() -> None:
    """Below FRONT_PIVOT_CM an arc cannot clear the wall: rotate in place
    toward the open side instead of grinding forward."""
    cmd_left = _step(front=FRONT_PIVOT_CM - 2, left=30.0, right=10.0)
    assert cmd_left.action == "pivot_left"
    cmd_right = _step(front=FRONT_PIVOT_CM - 2, left=10.0, right=30.0)
    assert cmd_right.action == "pivot_right"


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

    Prevents the over-correction loop (zigzag) the professor warned about.
    """
    half = DEADBAND_CM / 4  # so total error = DEADBAND_CM/2 < threshold
    cmd = _step(front=80.0, left=15.0 - half, right=15.0 + half)
    assert cmd.action == "arc"
    assert cmd.curvature == 0.0


def test_deadband_lets_real_errors_through() -> None:
    """Errors larger than DEADBAND_CM still produce a correction."""
    cmd = _step(front=80.0, left=10.0, right=10.0 + DEADBAND_CM * 5)
    assert cmd.curvature != 0.0


def test_centering_alone_never_saturates_steering() -> None:
    """Anti-zigzag: lane-keeping is capped at CENTERING_MAX even for huge
    offsets, so a noisy side reading cannot command a full-lock swerve.
    (Corner anticipation and the guards may still saturate.)"""
    ctrl = WallFollowController()
    cmd = None
    for _ in range(8):  # let the slew ramp fully settle
        cmd = ctrl.step(80.0, 5.0, 30.0)
    assert cmd is not None
    assert abs(cmd.curvature) <= CENTERING_MAX + 1e-9
    assert abs(cmd.curvature) >= CENTERING_MAX - 0.1  # actually settled there


# ---------- Wall-reacquire turn (U-turn fix) -----------------------------


def test_reacquire_triggers_on_confirmed_valid_opening() -> None:
    """Followed right wall replaced by sustained valid-far readings ->
    reacquire starts: first a straight pass-the-wall-end phase, then the
    closed-loop right turn."""
    ctrl = _armed_controller()
    ctrl.step(80.0, 15.0, REACQ_OPEN_CM + 10)          # open tick 1 — not yet
    cmd = ctrl.step(80.0, 15.0, REACQ_OPEN_CM + 10)    # open tick 2 — confirmed
    assert cmd.action == "arc"
    assert cmd.curvature == 0.0          # straight phase: clear the wall end
    assert cmd.linear_speed == REACQ_SPEED
    assert "reacquire_right" in cmd.reason
    # After the straight phase the wrap turn engages.
    for _ in range(REACQ_STRAIGHT_TICKS):
        cmd = ctrl.step(80.0, 15.0, REACQ_OPEN_CM + 10)
    assert "reacquire_right" in cmd.reason
    assert cmd.curvature == REACQ_CURVATURE


def test_reacquire_triggers_on_long_none_streak() -> None:
    """Real openings at 45 deg often return NO echo at all. A long None
    streak after wall-following (front open) must also start the maneuver."""
    ctrl = _armed_controller()
    cmd = None
    for _ in range(REACQ_NONE_TICKS):
        cmd = ctrl.step(80.0, 15.0, None)
    assert cmd is not None
    assert "reacquire_right" in cmd.reason
    for _ in range(REACQ_STRAIGHT_TICKS):
        cmd = ctrl.step(80.0, 15.0, None)
    assert cmd.curvature == REACQ_CURVATURE


def test_brief_none_burst_does_not_trigger_reacquire() -> None:
    """Short dropouts (the buried-sensor noise) hold the last wall reading
    and cause NO phantom swerve — the old phantom-400 zigzag source."""
    ctrl = _armed_controller()
    for _ in range(REACQ_NONE_TICKS - 1):
        cmd = ctrl.step(80.0, 15.0, None)
        assert "reacquire" not in cmd.reason
        assert abs(cmd.curvature) <= CENTERING_MAX + 1e-9


def test_single_noisy_far_echo_does_not_trigger() -> None:
    """One bad 'wall vanished' echo must not start a committed turn, and
    the PD lurch it would cause is capped by the per-tick curvature slew."""
    ctrl = _armed_controller()
    before = ctrl.step(80.0, 15.0, 20.0)
    mid = ctrl.step(80.0, 15.0, 380.0)   # single phantom-open tick
    after = ctrl.step(80.0, 15.0, 20.0)  # reading recovers
    assert "reacquire" not in mid.reason
    assert "reacquire" not in after.reason
    assert abs(mid.curvature - before.curvature) <= MAX_CURVATURE_STEP + 1e-9


def test_reacquire_holds_until_wall_found() -> None:
    """Mid-maneuver readings (no wall yet) keep the maneuver locked."""
    ctrl = _armed_controller()
    ctrl.step(80.0, 15.0, REACQ_OPEN_CM + 10)
    ctrl.step(80.0, 15.0, REACQ_OPEN_CM + 10)          # trigger
    cmd = None
    for _ in range(REACQ_STRAIGHT_TICKS + 3):           # into the turn phase
        cmd = ctrl.step(120.0, 50.0, None)              # rotating, nothing yet
    assert cmd is not None
    assert "reacquire_right" in cmd.reason
    assert cmd.curvature == REACQ_CURVATURE


def test_reacquire_ends_when_wall_reacquired() -> None:
    """Seeing the right wall again (valid, close) ends the turn and hands
    control back to the PD — closed loop, not a fixed-duration arc."""
    ctrl = _armed_controller()
    ctrl.step(80.0, 15.0, REACQ_OPEN_CM + 10)
    ctrl.step(80.0, 15.0, REACQ_OPEN_CM + 10)          # trigger
    ctrl.step(80.0, 15.0, 25.0)                         # wall! (streak 1)
    cmd = ctrl.step(80.0, 15.0, 25.0)                   # wall confirmed
    assert "reacquire" not in cmd.reason
    assert cmd.action == "arc"


def test_reacquire_aborts_when_front_blocks() -> None:
    """A front wall during the turn hands over to corner logic."""
    ctrl = _armed_controller()
    ctrl.step(80.0, 15.0, REACQ_OPEN_CM + 10)
    ctrl.step(80.0, 15.0, REACQ_OPEN_CM + 10)          # trigger
    cmd = ctrl.step(REACQ_FRONT_ABORT_CM - 5, 15.0, None)
    assert "reacquire" not in cmd.reason


def test_reacquire_times_out() -> None:
    """A turn that never finds a wall ends after REACQ_MAX_TICKS."""
    ctrl = _armed_controller()
    ctrl.step(80.0, 15.0, REACQ_OPEN_CM + 10)
    ctrl.step(80.0, 15.0, REACQ_OPEN_CM + 10)          # trigger (tick 1)
    cmd = None
    for _ in range(REACQ_MAX_TICKS + 1):
        cmd = ctrl.step(80.0, 15.0, REACQ_OPEN_CM + 10)
    assert cmd is not None
    assert "reacquire" not in cmd.reason


def test_left_opening_is_ignored_right_hand_rule() -> None:
    """Right-hand rule: a left opening while the right wall continues must
    NOT swerve the car left (the old left junction-commit did, and crashed
    in turnaround pockets). Centering may drift left, but only capped."""
    ctrl = _armed_controller()
    cmd = None
    for _ in range(4):
        cmd = ctrl.step(80.0, REACQ_OPEN_CM + 20, 20.0)
    assert cmd is not None
    assert "reacquire" not in cmd.reason
    assert cmd.curvature <= CENTERING_MAX + 1e-9  # no full-lock left swerve


# ---------- D-term clamping -------------------------------------------


def test_derror_clamped_on_big_sensor_jump() -> None:
    """The D-term contribution stays bounded even with huge raw derror."""
    ctrl = WallFollowController()
    ctrl.step(80.0, 15.0, 15.0)
    cmd = ctrl.step(80.0, 15.0, 35.0)
    assert "de=" in cmd.reason
    de_str = cmd.reason.split("de=")[1].split(" ")[0]
    de_val = float(de_str.rstrip("+"))
    assert abs(de_val) <= MAX_DERROR_CM + 1e-6


# ---------- Error-magnitude speed scaling -----------------------------


def test_large_error_reduces_speed_but_not_below_floor() -> None:
    """Big offset -> speed scales down but never below SPEED_SCALE_FLOOR."""
    cmd_big = _step(front=80.0, left=5.0, right=REACQ_OPEN_CM - 2)
    cmd_centered = _step(front=80.0, left=15.0, right=15.0)
    if cmd_big.linear_speed != cmd_centered.linear_speed:
        assert cmd_big.linear_speed < cmd_centered.linear_speed
    assert cmd_big.linear_speed >= BASE_SPEED * SPEED_SCALE_FLOOR - 0.01


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


def test_corner_direction_ignores_brief_side_dropout() -> None:
    """A one-tick side dropout at the corner must NOT flip the turn toward
    the (still present) dropped-out wall — the held value keeps the real
    geometry. This was a 30%-noise sim failure: l None'd out at the corner
    and the car turned left into the left wall."""
    ctrl = WallFollowController()
    ctrl.step(80.0, 28.0, 31.0)                            # both walls seen
    cmd = ctrl.step(CORNER_ANTICIPATE_CM - 10, None, 31.0)  # left drops out
    assert cmd.curvature < 0  # still turns toward the real opening (right)


# ---------- None / dropout policy --------------------------------------


def test_none_inputs_default_to_far() -> None:
    """All None with no history -> open corridor -> near-straight arc."""
    cmd = _step(front=None, left=None, right=None)
    assert cmd.action == "arc"
    assert abs(cmd.curvature) < 0.1


def test_side_none_never_reads_as_phantom_open_to_pd() -> None:
    """A None burst on one side holds the last valid wall distance until
    dead-sensor mirroring — there is no window where the side reads
    400/open and yanks the car toward the (still present) wall."""
    ctrl = WallFollowController()
    ctrl.step(80.0, 15.0, 15.0)
    # left None burst: left is held at 15, error stays ~0, no swerve.
    for _ in range(DEAD_SENSOR_TICKS + 2):
        cmd = ctrl.step(80.0, None, 15.0)
        assert cmd.action == "arc"
        assert abs(cmd.curvature) < 0.05


def test_front_none_holds_corner_caution() -> None:
    """A front dropout right before a corner must not cancel the corner
    slowdown/bias (old behavior: None -> 400 -> full cruise at the wall)."""
    ctrl = WallFollowController()
    first = ctrl.step(20.0, 15.0, 15.0)
    held = ctrl.step(None, 15.0, 15.0)
    assert held.linear_speed <= first.linear_speed + 1e-9   # no speed burst
    assert held.curvature <= first.curvature                # still cornering


def test_front_unknown_caps_speed() -> None:
    """Sustained front None = blind. The default flips to open for
    steering, but speed is capped at APPROACH_SPEED and the accel slew
    forbids any burst."""
    ctrl = WallFollowController()
    prev = ctrl.step(CORNER_FLOOR_CM + 1, 15.0, 15.0)
    cmd = prev
    for _ in range(FRONT_NONE_HOLD_TICKS + 4):
        cmd = ctrl.step(None, 15.0, 15.0)
        assert cmd.linear_speed <= prev.linear_speed + ACCEL_PWM_PER_TICK + 1e-9
        prev = cmd
    assert cmd.linear_speed <= APPROACH_SPEED + 1e-9


# ---------- Actuation slew (anti-lurch) ---------------------------------


def test_curvature_growth_is_rate_limited() -> None:
    """A sudden large error cannot saturate steering in one tick."""
    ctrl = WallFollowController()
    ctrl.step(80.0, 15.0, 15.0)          # straight
    cmd = ctrl.step(80.0, 5.0, 30.0)     # sudden 25cm offset
    assert abs(cmd.curvature) <= MAX_CURVATURE_STEP + 1e-9


def test_curvature_relaxes_to_straight_instantly() -> None:
    """Returning toward zero is never rate-limited (safe direction)."""
    ctrl = WallFollowController()
    ctrl.step(80.0, 15.0, 15.0)
    ctrl.step(80.0, 5.0, 30.0)           # steering grows
    cmd = ctrl.step(80.0, 15.0, 15.0)    # error gone
    assert cmd.curvature == 0.0


def test_speed_brakes_instantly_accelerates_gradually() -> None:
    ctrl = WallFollowController()
    open_cmd = ctrl.step(200.0, 15.0, 15.0)
    brake = ctrl.step(CORNER_FLOOR_CM + 1, 15.0, 15.0)
    assert brake.linear_speed < open_cmd.linear_speed - 5  # instant braking
    accel = ctrl.step(200.0, 15.0, 15.0)
    assert accel.linear_speed <= brake.linear_speed + ACCEL_PWM_PER_TICK + 1e-9


# ---------- Wide-area cap ----------------------------------------------


def test_wide_open_area_caps_speed_below_cruise() -> None:
    """Both sides far -> no lateral reference -> hold below full cruise.
    This is the 'speeds up across the open pocket and rams the far wall'
    field-bug defense."""
    cmd = _step(front=300.0, left=WIDE_OPEN_CM + 20, right=WIDE_OPEN_CM + 20)
    assert cmd.action == "arc"
    assert cmd.linear_speed <= CRUISE * WIDE_AREA_SPEED_FRACTION + 1e-9


def test_normal_corridor_not_wide_capped() -> None:
    cmd = _step(front=300.0, left=15.0, right=15.0)
    assert cmd.linear_speed == BASE_SPEED


# ---------- Speed table derives from one CRUISE knob --------------------


def test_speed_table_derives_from_cruise() -> None:
    """The whole table is fractions * CRUISE — one hardware-day dial."""
    assert BASE_SPEED == CRUISE
    assert APPROACH_SPEED == CRUISE * CORNER_APPROACH_FRACTION
    assert SLOW_SPEED == CRUISE * SLOW_FRACTION
    assert 0 < SLOW_FRACTION < CORNER_APPROACH_FRACTION < 1.0


def test_speed_names_remain_importable() -> None:
    """SLOW_SPEED / APPROACH_SPEED are referenced at runtime by the
    clearance guards + caps; pin them positive."""
    assert SLOW_SPEED > 0
    assert APPROACH_SPEED > 0
    assert CRUISE > 0
    assert REACQ_SPEED > 0


# ---------- Continuous front-clearance speed profile ---------------------


def test_front_speed_full_cruise_on_open_front() -> None:
    assert front_speed(200.0, CRUISE) == CRUISE


def test_front_speed_floor_is_positive() -> None:
    """Smooth-drive defense: never a zero-speed stall mid-corner."""
    assert front_speed(0.0, CRUISE) > 0.0


def test_front_speed_emits_intermediate_value_on_ramp() -> None:
    onset = corner_anticipate_cm(CRUISE)
    mid = (CORNER_FLOOR_CM + onset) / 2.0
    floor = CRUISE * CORNER_APPROACH_FRACTION
    s = front_speed(mid, CRUISE)
    assert floor < s < CRUISE


def test_front_speed_monotonic_nondecreasing() -> None:
    prev = -1.0
    f = 0.0
    while f <= 200.0:
        s = front_speed(f, CRUISE)
        assert s >= prev - 1e-9
        prev = s
        f += 1.0


def test_front_speed_endpoints_symbolic() -> None:
    c = 70.0
    assert front_speed(500.0, c) == c
    assert front_speed(CORNER_FLOOR_CM - 1.0, c) == c * CORNER_APPROACH_FRACTION


def test_step_open_straight_runs_at_cruise() -> None:
    """Centered + far front -> full cruise speed through the controller."""
    cmd = _step(front=200.0, left=15.0, right=15.0)
    assert cmd.action == "arc"
    assert cmd.linear_speed == BASE_SPEED


def test_narrowing_caps_speed_through_step() -> None:
    """Both sides tight (far front) -> speed capped at SLOW_SPEED."""
    cmd = _step(front=200.0, left=NARROWING_CM - 4, right=NARROWING_CM - 4)
    assert cmd.action == "arc"
    assert cmd.linear_speed <= SLOW_SPEED + 1e-9


# ---------- Speed-aware corner anticipation coupling ---------------------


def test_corner_anticipate_reference_value() -> None:
    """At the reference cruise the onset equals the configured reference
    lead; the runtime alias matches the current CRUISE."""
    assert corner_anticipate_cm(CRUISE_REF) == CORNER_ANTICIPATE_REF_CM
    assert CORNER_ANTICIPATE_CM == corner_anticipate_cm(CRUISE)


def test_corner_anticipate_couples_to_cruise() -> None:
    """A faster cruise begins cornering at a strictly LARGER distance."""
    assert corner_anticipate_cm(2 * CRUISE_REF) > corner_anticipate_cm(CRUISE_REF)


def test_ramp_onset_couples_to_cruise() -> None:
    """The speed-ramp onset and the corner-bias onset are the SAME cruise-
    coupled value."""
    onset_ref = corner_anticipate_cm(CRUISE_REF)
    onset_2x = corner_anticipate_cm(2 * CRUISE_REF)
    assert front_speed(onset_ref, CRUISE_REF) == CRUISE_REF
    assert front_speed(onset_ref - 0.5, CRUISE_REF) < CRUISE_REF
    assert front_speed(onset_2x, 2 * CRUISE_REF) == 2 * CRUISE_REF
    assert front_speed(onset_2x - 0.5, 2 * CRUISE_REF) < 2 * CRUISE_REF
    assert onset_2x > onset_ref


# ---------- Dead-side sensor defense ------------------------------------


def test_sustained_dead_left_does_not_steer_into_it() -> None:
    """After DEAD_SENSOR_TICKS consecutive None on the left, the left is
    mirrored to the live right -> the PD no longer steers into the blind
    left wall."""
    ctrl = WallFollowController()
    cmd = None
    for _ in range(DEAD_SENSOR_TICKS + 1):
        cmd = ctrl.step(80.0, None, 15.0)
    assert cmd is not None and cmd.action == "arc"
    assert cmd.curvature <= 0.1  # NOT steering left (+) into the dead side


def test_sustained_dead_right_does_not_steer_into_it() -> None:
    ctrl = WallFollowController()
    cmd = None
    for _ in range(DEAD_SENSOR_TICKS + 1):
        cmd = ctrl.step(80.0, 15.0, None)
    assert cmd is not None and cmd.action == "arc"
    assert cmd.curvature >= -0.1  # NOT steering right (-) into the dead side


def test_alive_sides_unchanged_by_dead_sensor_logic() -> None:
    """Both sides finite -> streaks stay 0 -> no mirroring -> normal PD
    (regression guard)."""
    cmd = _step(front=80.0, left=10.0, right=20.0)
    assert cmd.action == "arc"
    assert cmd.curvature < 0  # more room on right -> curve right, as before
