"""Behavioral tests for ``algorithm.wall_follower_sm.WallFollowerSM``.

Locks the safety semantics (RED stop, GREEN release, UNKNOWN holds when
stopped) and the boot/recovery transitions.
"""

from __future__ import annotations

from algorithm.wall_follower_sm import (
    EXIT_HOLD_TICKS,
    EXIT_OPEN_CM,
    INIT_WALL_FOUND_CM,
    PIVOT_EXIT_FRONT_CM,
    PIVOT_EXIT_HOLD_TICKS,
    PIVOT_MAX_TICKS,
    PIVOT_RECOVER_TICKS,
    WallFollowerSM,
)


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


def test_starting_in_open_space_does_not_count_as_exit() -> None:
    """The exit detector is disabled until the car has found maze walls."""
    sm = WallFollowerSM()
    for _ in range(EXIT_HOLD_TICKS + 2):
        cmd = sm.step(None, None, None, "UNKNOWN")
    assert sm.state == "INITIALIZING"
    assert cmd.action == "forward"
    assert not sm.done


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


# ---------- Exit detection ---------------------------------------------


def test_sustained_open_space_after_wall_found_exits() -> None:
    """After entering the maze, all three sensors open for long enough means
    the car has left the corridor and should stop instead of waiting for the
    duration cap."""
    sm = WallFollowerSM()
    _walk_to_following(sm)
    cmd = None
    for _ in range(EXIT_HOLD_TICKS):
        cmd = sm.step(EXIT_OPEN_CM + 20, None, None, "UNKNOWN")
    assert sm.state == "EXITED"
    assert sm.done
    assert cmd is not None
    assert cmd.action == "stop"


def test_red_stop_suppresses_exit_detection_until_green() -> None:
    sm = WallFollowerSM()
    _walk_to_following(sm)
    sm.step(EXIT_OPEN_CM + 20, None, None, "STOP")
    for _ in range(EXIT_HOLD_TICKS + 2):
        cmd = sm.step(EXIT_OPEN_CM + 20, None, None, "UNKNOWN")
    assert sm.state == "STOPPED_AT_RED"
    assert cmd.action == "stop"
    assert not sm.done


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


# ---------- Full-chain speed profile (SM -> controller integration) ----


def test_full_chain_open_straight_faster_than_corner_approach() -> None:
    """End-to-end through the state machine + continuous speed profile: an
    open straight runs faster than the same car approaching a front wall.
    Exercises the SPEED-1 front-clearance ramp via the real SM path (which
    main.py --dry-run cannot, since it never leaves INITIALIZING)."""
    sm = WallFollowerSM()
    _walk_to_following(sm)
    open_cmd = sm.step(200.0, 15.0, 15.0, "UNKNOWN")
    corner_cmd = sm.step(15.0, 15.0, 30.0, "UNKNOWN")  # front wall near; r<40 => no commit
    assert open_cmd.action == "arc"
    assert corner_cmd.action == "arc"
    assert open_cmd.linear_speed > corner_cmd.linear_speed


# ---------- Infinite-pivot bailout (SM-1) ------------------------------


def test_stuck_pivot_caps_into_reverse_escape() -> None:
    """A never-clearing dead-end must NOT pivot forever: after
    PIVOT_MAX_TICKS it bails into RECOVERING with a reverse (backward)
    escape, not endless in-place pivoting."""
    sm = WallFollowerSM()
    _walk_to_following(sm)
    actions = [
        sm.step(5.0, 5.0, 5.0, "UNKNOWN").action
        for _ in range(PIVOT_MAX_TICKS + 1)
    ]
    assert "backward" in actions  # bailed out, did not pivot forever
    assert sm.state == "RECOVERING"


def test_reverse_escape_leaves_recovering_and_re_evaluates() -> None:
    """After the reverse-escape window the SM leaves RECOVERING and resumes
    evaluation (here it re-pivots since the inputs stay boxed) — it must not
    stay stuck in RECOVERING. Verifies real forward progress, not a single
    transition tick that re-enters the stuck state."""
    sm = WallFollowerSM()
    _walk_to_following(sm)
    for _ in range(PIVOT_MAX_TICKS + 1):
        sm.step(5.0, 5.0, 5.0, "UNKNOWN")
    assert sm.state == "RECOVERING"
    states = [
        (sm.step(5.0, 5.0, 5.0, "UNKNOWN"), sm.state)[1]
        for _ in range(PIVOT_RECOVER_TICKS + 2)
    ]
    assert "RECOVERING" not in states[-1:]  # left RECOVERING by the end
    assert "PIVOTING" in states or "FOLLOWING" in states


def test_red_preempts_reverse_escape() -> None:
    """RED during RECOVERING still forces an immediate STOP (safety wins)."""
    sm = WallFollowerSM()
    _walk_to_following(sm)
    for _ in range(PIVOT_MAX_TICKS + 1):
        sm.step(5.0, 5.0, 5.0, "UNKNOWN")
    assert sm.state == "RECOVERING"
    cmd = sm.step(5.0, 5.0, 5.0, "STOP")
    assert sm.state == "STOPPED_AT_RED"
    assert cmd.action == "stop"


def test_full_chain_red_stops_then_green_resumes_driving() -> None:
    """Traffic-light gating still wraps the new speed profile correctly:
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
