"""Top-level state machine: right-hand wall follower with traffic-light gating.

States
------
  INITIALIZING        - boot; finding a wall to follow (drive forward slowly
                        until a side wall appears).
  FOLLOWING           - normal smooth wall-following via WallFollowController.
  STOPPED_AT_RED      - perception said STOP; brakes locked until explicit GREEN.
  PIVOTING            - pivot fallback for tight dead-ends; held until front clears.

Transition rules (per CLAUDE.md):
  - RED -> STOPPED_AT_RED from any moving state.
  - In STOPPED_AT_RED, only explicit GREEN releases the brake. UNKNOWN keeps
    us stopped (safety-asymmetry rule).
  - In moving states, GREEN and UNKNOWN both mean "keep going".
  - If WallFollowController.step() returns action='pivot_*', we enter PIVOTING.
  - In PIVOTING, exit back to FOLLOWING when front_cm > PIVOT_EXIT_FRONT_CM
    sustained for PIVOT_EXIT_HOLD_TICKS.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Literal

from logs.trace import tracer

from control.wall_follow import WallFollowCommand, WallFollowController

# Redefined locally (mirrors perception.traffic_light.Signal) so this
# module is importable off the Pi where cv2 isn't installed — needed for
# `python main.py --dry-run` on dev machines.
Signal = Literal["STOP", "GO", "UNKNOWN"]
State = Literal["INITIALIZING", "FOLLOWING", "STOPPED_AT_RED", "PIVOTING", "RECOVERING"]

INIT_WALL_FOUND_CM = 30.0
PIVOT_EXIT_FRONT_CM = 18.0
PIVOT_EXIT_HOLD_TICKS = 5

# Infinite-pivot bailout: if PIVOTING runs this many ticks without the front
# clearing, the dead-end isn't resolving (the car isn't rotating free), so we
# bail into RECOVERING. ~6s at 10Hz is far longer than a real ~1s 180° pivot,
# so this only fires on a genuinely stuck pivot. Pi-tunable + Pi-validate.
PIVOT_MAX_TICKS = 60
# RECOVERING reverses slowly for this many ticks to back out of the pocket
# (a dead-end's exit is behind the car), then returns to FOLLOWING to
# re-evaluate. Reverse (not forward) so we never drive into the dead-end wall.
PIVOT_RECOVER_TICKS = 5
PIVOT_RECOVER_SPEED = 30.0

INIT_FORWARD_SPEED = 30.0


@dataclass(frozen=True)
class HighLevelCommand:
    action: Literal["arc", "pivot_right", "pivot_left", "forward", "backward", "stop"]
    linear_speed: float = 0.0
    curvature: float = 0.0
    reason: str = ""


class WallFollowerSM:
    def __init__(self) -> None:
        self._state: State = "INITIALIZING"
        self._controller = WallFollowController()
        self._pivot_clear_ticks = 0
        self._pivot_ticks = 0       # total ticks in the current PIVOTING episode
        self._recover_ticks = 0     # ticks elapsed in RECOVERING reverse-escape
        tracer.state(state=self._state, from_state=None, reason="boot")

    @property
    def state(self) -> State:
        return self._state

    def step(
        self,
        front_cm: float | None,
        left_cm: float | None,
        right_cm: float | None,
        signal: Signal,
    ) -> HighLevelCommand:
        # Traffic-light gating (highest priority).
        if signal == "STOP" and self._state != "STOPPED_AT_RED":
            self._transition("STOPPED_AT_RED", reason="perception: STOP")
        elif self._state == "STOPPED_AT_RED" and signal == "GO":
            self._transition("FOLLOWING", reason="perception: explicit GREEN releases brake")

        if self._state == "STOPPED_AT_RED":
            return HighLevelCommand(action="stop", reason="STOPPED_AT_RED")

        # Reverse-escape after a stuck pivot. Slow reverse for a few ticks to
        # back out of a dead-end pocket, then re-evaluate from FOLLOWING (fall
        # through to the controller below). RED still preempts via the gate
        # above, so this never overrides a stop.
        if self._state == "RECOVERING":
            self._recover_ticks += 1
            if self._recover_ticks > PIVOT_RECOVER_TICKS:
                self._transition("FOLLOWING", reason="reverse-escape done")
            else:
                return HighLevelCommand(
                    action="backward",
                    linear_speed=PIVOT_RECOVER_SPEED,
                    reason=f"reverse escape ({self._recover_ticks}/{PIVOT_RECOVER_TICKS})",
                )

        # Init: drive forward until wall detected on either side.
        if self._state == "INITIALIZING":
            if (left_cm is not None and left_cm < INIT_WALL_FOUND_CM) or (
                right_cm is not None and right_cm < INIT_WALL_FOUND_CM
            ):
                self._transition("FOLLOWING", reason=f"wall found (l={left_cm}, r={right_cm})")
            else:
                return HighLevelCommand(
                    action="forward",
                    linear_speed=INIT_FORWARD_SPEED,
                    reason="searching for wall",
                )

        # Wall-follow controller.
        wf_cmd = self._controller.step(front_cm, left_cm, right_cm)

        if wf_cmd.action.startswith("pivot"):
            if self._state != "PIVOTING":
                self._transition("PIVOTING", reason=wf_cmd.reason)
                self._pivot_ticks = 0
            self._pivot_clear_ticks = 0
            self._pivot_ticks += 1
            # Infinite-pivot bailout: a never-clearing dead-end can't pivot
            # forever — bail into a reverse-escape instead of spinning in place.
            if self._pivot_ticks >= PIVOT_MAX_TICKS:
                self._transition(
                    "RECOVERING",
                    reason=f"pivot stuck {self._pivot_ticks} ticks -> reverse escape",
                )
                self._recover_ticks = 0
                return HighLevelCommand(
                    action="backward",
                    linear_speed=PIVOT_RECOVER_SPEED,
                    reason="pivot stuck -> reverse escape",
                )
            return _wf_to_high(wf_cmd)

        if self._state == "PIVOTING":
            if (front_cm is None) or (front_cm > PIVOT_EXIT_FRONT_CM):
                self._pivot_clear_ticks += 1
            else:
                self._pivot_clear_ticks = 0
            if self._pivot_clear_ticks >= PIVOT_EXIT_HOLD_TICKS:
                self._transition("FOLLOWING", reason="pivot exit: front cleared")
            else:
                return HighLevelCommand(
                    action="pivot_right",
                    linear_speed=30.0,
                    reason=f"continuing pivot (clear ticks {self._pivot_clear_ticks})",
                )

        return _wf_to_high(wf_cmd)

    # ------------------------------------------------------------------ #
    def _transition(self, new: State, reason: str) -> None:
        old = self._state
        self._state = new
        tracer.state(state=new, from_state=old, reason=reason)


def _wf_to_high(cmd: WallFollowCommand) -> HighLevelCommand:
    return HighLevelCommand(
        action=cmd.action,  # type: ignore[arg-type]
        linear_speed=cmd.linear_speed,
        curvature=cmd.curvature,
        reason=cmd.reason,
    )
