"""Behavioral tests for ``algorithm.wall_follower_sm.WallFollowerSM``.

Locks the safety semantics (RED stop, GREEN release, UNKNOWN holds when
stopped), the boot/recovery transitions, and the None-robust stuck/pivot
judgments. There is intentionally no self-stop / exit detection (removed
2026-06-10): the open-space test below pins "keep driving" behavior.
"""

from __future__ import annotations

from algorithm.wall_follower_sm import (
    FRONT_HOLD_TICKS,
    INIT_FRONT_BLOCK_CM,
    INIT_WALL_FOUND_CM,
    PIVOT_EXIT_FRONT_CM,
    PIVOT_EXIT_HOLD_TICKS,
    PIVOT_MAX_TICKS,
    RECOVER_REVERSE_TICKS,
    RECOVER_TURN_TICKS,
    STUCK_TRIGGER_TICKS,
    WallFollowerSM,
)
from control.wall_follow import FRONT_PIVOT_CM


# ---------- INIT -> FOLLOWING -----------------------------------------


def test_initializing_stays_when_no_walls() -> None:
    sm = WallFollowerSM()
    cmd = sm.step(None, None, None, "UNKNOWN")
    assert sm.state == "INITIALIZING"
    assert cmd.action == "forward"


def test_initializing_advances_when_right_wall_found() -> None:
    sm = WallFollowerSM()
    sm.step(None, None, INIT_WALL_FOUND_CM - 5, "UNKNOWN")
    assert sm.state == "FOLLOWING"


def test_initializing_advances_when_left_wall_found() -> None:
    sm = WallFollowerSM()
    sm.step(None, INIT_WALL_FOUND_CM - 5, None, "UNKNOWN")
    assert sm.state == "FOLLOWING"


def test_initializing_front_blocked_rotates_instead_of_forward() -> None:
    """Booting nose-first into a wall must not launch the car forward."""
    sm = WallFollowerSM()
    cmd = sm.step(INIT_FRONT_BLOCK_CM - 5, None, None, "UNKNOWN")
    assert sm.state == "INITIALIZING"
    assert cmd.action == "pivot_right"


def test_initializing_open_space_keeps_searching() -> None:
    """No walls anywhere: keep searching forward; never self-stop."""
    sm = WallFollowerSM()
    for _ in range(40):
        cmd = sm.step(None, None, None, "UNKNOWN")
        assert cmd.action == "forward"
    assert sm.state == "INITIALIZING"


# ---------- Traffic-light safety semantics -----------------------------


def _walk_to_following(sm: WallFollowerSM) -> None:
    sm.step(80.0, 15.0, 15.0, "UNKNOWN")
    assert sm.state == "FOLLOWING"


def test_red_from_following_stops_immediately() -> None:
    sm = WallFollowerSM()
    _walk_to_following(sm)
    cmd = sm.step(80.0, 15.0, 15.0, "STOP")
    assert sm.state == "STOPPED_AT_RED"
    assert cmd.action == "stop"


def test_unknown_holds_stop() -> None:
    sm = WallFollowerSM()
    _walk_to_following(sm)
    sm.step(80.0, 15.0, 15.0, "STOP")
    cmd = sm.step(80.0, 15.0, 15.0, "UNKNOWN")
    assert sm.state == "STOPPED_AT_RED"
    assert cmd.action == "stop"


def test_green_releases_stop() -> None:
    sm = WallFollowerSM()
    _walk_to_following(sm)
    sm.step(80.0, 15.0, 15.0, "STOP")
    cmd = sm.step(80.0, 15.0, 15.0, "GO")
    assert sm.state == "FOLLOWING"
    assert cmd.action == "arc"


def test_green_in_following_is_noop_keeps_driving() -> None:
    sm = WallFollowerSM()
    _walk_to_following(sm)
    cmd = sm.step(80.0, 15.0, 15.0, "GO")
    assert sm.state == "FOLLOWING"
    assert cmd.action == "arc"


# ---------- No self-stop (exit auto-detection removed) ------------------


def test_open_space_never_self_stops() -> None:
    """THE field bug this replaces: wide turnaround pockets used to be
    misread as 'maze exited' and the car stopped mid-run. There is no exit
    detector anymore — open readings just mean keep driving."""
    sm = WallFollowerSM()
    _walk_to_following(sm)
    for _ in range(80):
        cmd = sm.step(200.0, 180.0, 180.0, "UNKNOWN")
        assert cmd.action != "stop"
    assert sm.state == "FOLLOWING"


def test_none_bursts_never_self_stop() -> None:
    """All-None stretches (buried sensors) must also keep the run alive."""
    sm = WallFollowerSM()
    _walk_to_following(sm)
    for _ in range(60):
        cmd = sm.step(None, None, None, "UNKNOWN")
        assert cmd.action != "stop"


# ---------- Pivot / dead-end ------------------------------------------


def test_dead_end_enters_pivoting() -> None:
    sm = WallFollowerSM()
    _walk_to_following(sm)
    cmd = sm.step(5.0, 5.0, 5.0, "UNKNOWN")
    assert sm.state == "PIVOTING"
    assert cmd.action.startswith("pivot")


def test_pivoting_exits_after_sustained_clear_front() -> None:
    sm = WallFollowerSM()
    _walk_to_following(sm)
    sm.step(5.0, 5.0, 5.0, "UNKNOWN")
    assert sm.state == "PIVOTING"
    for _ in range(PIVOT_EXIT_HOLD_TICKS):
        sm.step(PIVOT_EXIT_FRONT_CM + 5, 15.0, 15.0, "UNKNOWN")
    assert sm.state == "FOLLOWING"


def test_pivot_exit_not_fooled_by_front_none_burst() -> None:
    """A brief front None while pivoting is a dropout, NOT 'front cleared'.
    The old code counted None as clear and ended pivots still facing the
    wall — the thrash-at-the-wall field bug."""
    sm = WallFollowerSM()
    _walk_to_following(sm)
    sm.step(5.0, 5.0, 5.0, "UNKNOWN")
    assert sm.state == "PIVOTING"
    for _ in range(min(PIVOT_EXIT_HOLD_TICKS + 1, FRONT_HOLD_TICKS - 1)):
        sm.step(None, 5.0, 5.0, "UNKNOWN")
    assert sm.state == "PIVOTING"


def test_pivot_direction_preserved_from_controller() -> None:
    """The controller picks the pivot direction (more open side); the SM
    must keep commanding THAT direction, not hardcode pivot_right."""
    sm = WallFollowerSM()
    _walk_to_following(sm)
    first = sm.step(FRONT_PIVOT_CM - 2, 30.0, 10.0, "UNKNOWN")
    assert sm.state == "PIVOTING"
    assert first.action == "pivot_left"
    second = sm.step(FRONT_PIVOT_CM - 2, 30.0, 10.0, "UNKNOWN")
    assert second.action == "pivot_left"


def test_jammed_pivot_bails_into_recovery() -> None:
    """A pivot that never clears the front (front hovering above the stuck
    line but below the exit line) must not spin forever: after
    PIVOT_MAX_TICKS it bails into the reverse escape."""
    sm = WallFollowerSM()
    _walk_to_following(sm)
    sm.step(5.0, 5.0, 5.0, "UNKNOWN")
    assert sm.state == "PIVOTING"
    actions = []
    for _ in range(PIVOT_MAX_TICKS + 1):
        # 15cm: above STUCK_FRONT_CM (no stuck trigger), below
        # PIVOT_EXIT_FRONT_CM (pivot never 'clears').
        actions.append(sm.step(15.0, 5.0, 5.0, "UNKNOWN").action)
    assert sm.state == "RECOVERING"
    assert "backward" in actions


# ---------- Stuck detection + two-phase recovery ------------------------


def test_stuck_front_triggers_reverse_then_turn_recovery() -> None:
    """Front pinned close for STUCK_TRIGGER_TICKS -> reverse escape, then a
    pivot toward open space, then back to normal evaluation. The turn phase
    is what prevents driving straight back into the same wall (the flailing
    field bug)."""
    sm = WallFollowerSM()
    _walk_to_following(sm)
    actions, states = [], []
    total = STUCK_TRIGGER_TICKS + RECOVER_REVERSE_TICKS + RECOVER_TURN_TICKS + 3
    for _ in range(total):
        cmd = sm.step(10.0, 15.0, 15.0, "UNKNOWN")
        actions.append(cmd.action)
        states.append(sm.state)
    assert "RECOVERING" in states
    first_back = actions.index("backward")
    i = first_back
    while i < len(actions) and actions[i] == "backward":
        i += 1
    assert i < len(actions), "recovery should not end the trace"
    assert actions[i].startswith("pivot"), "reverse phase must chain into a turn"


def test_front_none_does_not_reset_stuck_evidence() -> None:
    """A sensor crushed against a wall returns None. The old code RESET the
    stuck counter on None, disabling stuck detection exactly when it was
    needed. Held/unknown front must keep the evidence."""
    sm = WallFollowerSM()
    _walk_to_following(sm)
    for _ in range(STUCK_TRIGGER_TICKS - 5):
        sm.step(10.0, 15.0, 15.0, "UNKNOWN")
    for _ in range(min(6, FRONT_HOLD_TICKS - 1)):       # None burst mid-evidence
        sm.step(None, 15.0, 15.0, "UNKNOWN")
        if sm.state == "RECOVERING":
            break
    for _ in range(STUCK_TRIGGER_TICKS):                # pinned again
        if sm.state == "RECOVERING":
            break
        sm.step(10.0, 15.0, 15.0, "UNKNOWN")
    assert sm.state == "RECOVERING"


def test_recovery_completes_and_reenters_evaluation() -> None:
    """After the reverse+turn window the SM leaves RECOVERING and resumes
    evaluation (here it re-pivots since the inputs stay boxed) — it must
    not stay stuck in RECOVERING."""
    sm = WallFollowerSM()
    _walk_to_following(sm)
    for _ in range(STUCK_TRIGGER_TICKS):
        sm.step(5.0, 5.0, 5.0, "UNKNOWN")
    assert sm.state == "RECOVERING"
    for _ in range(RECOVER_REVERSE_TICKS + RECOVER_TURN_TICKS + 1):
        sm.step(5.0, 5.0, 5.0, "UNKNOWN")
    assert sm.state in ("PIVOTING", "FOLLOWING")


def test_repeated_stuck_escalates_reverse_duration() -> None:
    """A second stuck episode shortly after the first reverses for LONGER —
    the maneuver that just failed is not retried verbatim."""
    sm = WallFollowerSM()
    _walk_to_following(sm)
    actions = []
    for _ in range(90):
        actions.append(sm.step(10.0, 15.0, 15.0, "UNKNOWN").action)
    # Collect lengths of consecutive-backward runs.
    runs, current = [], 0
    for a in actions:
        if a == "backward":
            current += 1
        elif current:
            runs.append(current)
            current = 0
    if current:
        runs.append(current)
    assert len(runs) >= 2, "expected at least two recovery episodes"
    assert runs[1] > runs[0], "second reverse escape should escalate"


def test_red_preempts_reverse_escape() -> None:
    """RED during RECOVERING still forces an immediate STOP (safety wins)."""
    sm = WallFollowerSM()
    _walk_to_following(sm)
    for _ in range(STUCK_TRIGGER_TICKS):
        sm.step(5.0, 5.0, 5.0, "UNKNOWN")
    assert sm.state == "RECOVERING"
    cmd = sm.step(5.0, 5.0, 5.0, "STOP")
    assert sm.state == "STOPPED_AT_RED"
    assert cmd.action == "stop"


# ---------- Full-chain speed profile (SM -> controller integration) ----


def test_full_chain_open_straight_faster_than_corner_approach() -> None:
    """End-to-end through the state machine + continuous speed profile: an
    open straight runs faster than the same car approaching a front wall."""
    sm = WallFollowerSM()
    _walk_to_following(sm)
    open_cmd = sm.step(200.0, 15.0, 15.0, "UNKNOWN")
    corner_cmd = sm.step(15.0, 15.0, 30.0, "UNKNOWN")
    assert open_cmd.action == "arc"
    assert corner_cmd.action == "arc"
    assert open_cmd.linear_speed > corner_cmd.linear_speed


def test_full_chain_red_stops_then_green_resumes_driving() -> None:
    """Traffic-light gating still wraps the speed profile correctly:
    RED -> stop, explicit GREEN -> resume arc-ing at a positive speed."""
    sm = WallFollowerSM()
    _walk_to_following(sm)
    stop_cmd = sm.step(200.0, 15.0, 15.0, "STOP")
    assert sm.state == "STOPPED_AT_RED"
    assert stop_cmd.action == "stop"
    go_cmd = sm.step(200.0, 15.0, 15.0, "GO")
    assert sm.state == "FOLLOWING"
    assert go_cmd.action == "arc"
    assert go_cmd.linear_speed > 0
