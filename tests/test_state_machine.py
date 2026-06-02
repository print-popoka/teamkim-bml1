"""Behavioral tests for ``algorithm.wall_follower_sm.WallFollowerSM``.

Locks the safety semantics (RED stop, GREEN release, UNKNOWN holds when
stopped) and the boot/recovery transitions.
"""

from __future__ import annotations

from algorithm.wall_follower_sm import (
    INIT_WALL_FOUND_CM,
    PIVOT_EXIT_FRONT_CM,
    PIVOT_EXIT_HOLD_TICKS,
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
