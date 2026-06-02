"""Pure maze and demo decision logic — OFFLINE SIMULATOR ONLY.

Production main loop (``main.py``) does NOT use the controllers in this
file. The live wall-follower is ``algorithm/wall_follower_sm.py``
delegating to ``control/wall_follow.py`` (smooth PD wall-follow with
junction commit, corner anticipation, D-clamp, deadband). The classes
here exist for ``algorithm/simulate.py`` and ``tests/test_maze_logic.py``
to validate discrete-decision invariants (right-hand priority, dead-end
U-turn, RED latch, etc.) without GPIO/camera/OpenCV deps.

If a behavior change is needed in production, update the smooth-PD path
in ``control/wall_follow.py`` and ``algorithm/wall_follower_sm.py`` —
those are the ones ``main.py`` actually runs.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum


class Signal(str, Enum):
    STOP = "STOP"
    GO = "GO"
    UNKNOWN = "UNKNOWN"


class Action(str, Enum):
    STOP = "stop"
    FORWARD = "forward"
    TURN_RIGHT = "turn_right"
    TURN_LEFT = "turn_left"
    UTURN = "uturn"


@dataclass(frozen=True)
class SensorFrame:
    front_cm: float | None
    left_cm: float | None
    right_cm: float | None
    signal: Signal = Signal.UNKNOWN


@dataclass(frozen=True)
class Decision:
    action: Action
    left_pwm: float
    right_pwm: float
    state: str
    reason: str


@dataclass(frozen=True)
class MazeConfig:
    base_pwm: float = 38.0
    turn_pwm: float = 34.0
    slow_pwm: float = 24.0
    max_pwm: float = 55.0
    front_block_cm: float = 12.0
    side_wall_cm: float = 18.0
    side_open_cm: float = 24.0
    safe_side_cm: float = 5.0
    center_kp: float = 1.0


def _clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


def _signal(value: Signal | str) -> Signal:
    if isinstance(value, Signal):
        return value
    try:
        return Signal(value)
    except ValueError:
        return Signal.UNKNOWN


class RightHandMazeController:
    """Right-hand wall-following controller for a simply connected maze.

    Priority order after traffic-light handling:
    1. If front sensor is invalid, stop rather than drive blind.
    2. If too close to a side wall, steer away immediately.
    3. If front is blocked, choose right, then left, then U-turn.
    4. If the right side opens, turn right to keep the right-hand rule.
    5. Otherwise drive forward, centered by left/right distance difference.
    """

    def __init__(self, config: MazeConfig | None = None) -> None:
        self.config = config or MazeConfig()
        self.stopped_for_red = False

    def decide(self, frame: SensorFrame) -> Decision:
        cfg = self.config
        sig = _signal(frame.signal)

        if self.stopped_for_red:
            if sig == Signal.GO:
                self.stopped_for_red = False
            else:
                return Decision(Action.STOP, 0, 0, "STOPPED_RED", "waiting for explicit GO")

        if sig == Signal.STOP:
            self.stopped_for_red = True
            return Decision(Action.STOP, 0, 0, "STOPPED_RED", "red traffic light")

        if frame.front_cm is None:
            return Decision(Action.STOP, 0, 0, "SENSOR_FAIL", "front distance unavailable")

        front_blocked = frame.front_cm <= cfg.front_block_cm
        right_open = frame.right_cm is not None and frame.right_cm >= cfg.side_open_cm
        left_open = frame.left_cm is not None and frame.left_cm >= cfg.side_open_cm

        if frame.left_cm is not None and frame.left_cm < cfg.safe_side_cm:
            return Decision(Action.TURN_RIGHT, cfg.turn_pwm, -cfg.turn_pwm, "CLEARANCE", "left wall too close")
        if frame.right_cm is not None and frame.right_cm < cfg.safe_side_cm:
            return Decision(Action.TURN_LEFT, -cfg.turn_pwm, cfg.turn_pwm, "CLEARANCE", "right wall too close")

        if front_blocked:
            if right_open:
                return Decision(Action.TURN_RIGHT, cfg.turn_pwm, -cfg.turn_pwm, "TURN_RIGHT", "front blocked, right open")
            if left_open:
                return Decision(Action.TURN_LEFT, -cfg.turn_pwm, cfg.turn_pwm, "TURN_LEFT", "front blocked, left open")
            return Decision(Action.UTURN, cfg.turn_pwm, -cfg.turn_pwm, "UTURN", "dead end")

        if right_open:
            return Decision(Action.TURN_RIGHT, cfg.turn_pwm, -cfg.turn_pwm, "TURN_RIGHT", "right-hand opening")

        if frame.left_cm is None or frame.right_cm is None:
            return Decision(Action.FORWARD, cfg.slow_pwm, cfg.slow_pwm, "FORWARD_SLOW", "side distance unavailable")

        width = frame.left_cm + frame.right_cm
        base = cfg.slow_pwm if width < 25 else cfg.base_pwm
        error = frame.right_cm - frame.left_cm
        correction = _clamp(error * cfg.center_kp, -12, 12)
        left_pwm = _clamp(base + correction, 0, cfg.max_pwm)
        right_pwm = _clamp(base - correction, 0, cfg.max_pwm)
        return Decision(Action.FORWARD, left_pwm, right_pwm, "FOLLOW_WALL", "centered wall follow")


class GreenStopDemoController:
    """Demo-only controller: U-turn until GREEN is visible, then stop.

    This intentionally does not implement real traffic-light semantics.
    Keep it separate from RightHandMazeController so a demo rule cannot leak
    into the maze run where GREEN means release/continue.
    """

    def __init__(self, turn_pwm: float = 34.0) -> None:
        self.turn_pwm = turn_pwm
        self.stopped_on_green = False

    def decide(self, signal: Signal | str) -> Decision:
        sig = _signal(signal)
        if self.stopped_on_green:
            return Decision(Action.STOP, 0, 0, "DEMO_STOPPED_GREEN", "green already seen")
        if sig == Signal.GO:
            self.stopped_on_green = True
            return Decision(Action.STOP, 0, 0, "DEMO_STOPPED_GREEN", "green detected")
        return Decision(Action.UTURN, self.turn_pwm, -self.turn_pwm, "DEMO_UTURN", "waiting for green")
