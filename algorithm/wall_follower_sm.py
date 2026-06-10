"""Top-level state machine: right-hand wall follower with traffic-light gating.

States
------
  INITIALIZING        - boot; finding a wall to follow (drive forward slowly
                        until a side wall appears; rotate first if the front
                        is blocked).
  FOLLOWING           - normal smooth wall-following via WallFollowController.
  STOPPED_AT_RED      - perception said STOP; brakes locked until explicit GREEN.
  PIVOTING            - pivot fallback for tight dead-ends / head-on emergencies;
                        held until the front clears. Direction is chosen by the
                        controller (more open side) and preserved.
  RECOVERING          - two-phase escape: reverse out of the pocket, then pivot
                        toward the more open side, then resume FOLLOWING.
                        Repeat triggers escalate the reverse duration.

There is deliberately NO self-stop / maze-exit auto-detection (removed
2026-06-10, user decision): a false "exit detected" stop mid-maze kills the
run, while a true exit is handled by simply cutting power. The run ends via
Ctrl+C, --duration, or battery pull. STOPPED_AT_RED stays — reacting to the
red light is a graded requirement, not an emergency stop.

Transition rules (per CLAUDE.md):
  - RED -> STOPPED_AT_RED from any moving state.
  - In STOPPED_AT_RED, only explicit GREEN releases the brake. UNKNOWN keeps
    us stopped (safety-asymmetry rule).
  - In moving states, GREEN and UNKNOWN both mean "keep going".
  - If WallFollowController.step() returns action='pivot_*', we enter PIVOTING.
  - In PIVOTING, exit back to FOLLOWING when the HELD front clears
    PIVOT_EXIT_FRONT_CM sustained for PIVOT_EXIT_HOLD_TICKS. A brief front
    None does NOT count as clear (buried-sensor dropout), so the car cannot
    end a pivot still facing the wall — that was the "thrash at the wall"
    field bug.

Front-None policy: ``held front`` = last valid front while the dropout is
shorter than FRONT_HOLD_TICKS; after that it is unknown (None). The stuck
detector treats unknown as "no new evidence" (the count holds — a sensor
crushed against a wall typically returns None, which previously RESET the
counter and disabled stuck detection exactly when it was needed).
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
State = Literal[
    "INITIALIZING",
    "FOLLOWING",
    "STOPPED_AT_RED",
    "PIVOTING",
    "RECOVERING",
]

INIT_WALL_FOUND_CM = 30.0
# Booting nose-first into a wall: rotate instead of driving forward blind.
INIT_FRONT_BLOCK_CM = 15.0
INIT_FORWARD_SPEED = 30.0
INIT_TURN_SPEED = 30.0

PIVOT_EXIT_FRONT_CM = 18.0
PIVOT_EXIT_HOLD_TICKS = 5

# Infinite-pivot bailout: if PIVOTING runs this many ticks without the front
# clearing, the car isn't rotating free, so we bail into RECOVERING.
PIVOT_MAX_TICKS = 60

# Stuck detector: front pinned at/below STUCK_FRONT_CM for
# STUCK_TRIGGER_TICKS (~2 s) means we are grinding against something.
# Held-None front does not reset the count (see module docstring).
STUCK_FRONT_CM = 12.0
STUCK_TRIGGER_TICKS = 20

# Held-front horizon: a front None shorter than this returns the last valid
# value; longer dropouts are "unknown". Longer than the controller's hold
# (5) because the SM uses it for clear/stuck *judgments*, not steering.
FRONT_HOLD_TICKS = 8

# RECOVERING: phase 1 reverses out of the pocket (a dead-end's exit is
# behind the car), phase 2 pivots toward the more open side so we do not
# drive straight back into the same wall — that re-entry loop was the
# "flailing at the wall" field bug. Consecutive recoveries within
# RECOVER_MEMORY_TICKS escalate the reverse duration (same maneuver that
# just failed is not retried verbatim).
RECOVER_REVERSE_TICKS = 8
RECOVER_REVERSE_TICKS_MAX = 16
RECOVER_REVERSE_ESCALATION = 4
RECOVER_TURN_TICKS = 4
RECOVER_MEMORY_TICKS = 100
PIVOT_RECOVER_SPEED = 30.0
RECOVER_TURN_SPEED = 35.0


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
        self._pivot_action: str = "pivot_right"   # direction preserved from trigger
        self._has_found_wall = False
        self._stuck_ticks = 0       # ticks with front wall extremely close
        # Held-front bookkeeping (SM-level judgments).
        self._front_none_streak = 0
        self._last_valid_front: float | None = None
        # RECOVERING bookkeeping.
        self._recover_ticks = 0
        self._recover_reverse_target = RECOVER_REVERSE_TICKS
        self._recover_dir: Literal["pivot_left", "pivot_right"] = "pivot_right"
        self._recover_attempts = 0
        self._ticks_since_recover = RECOVER_MEMORY_TICKS + 1
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
        held_front = self._update_held_front(front_cm)

        # Traffic-light gating (highest priority).
        if signal == "STOP" and self._state != "STOPPED_AT_RED":
            self._transition("STOPPED_AT_RED", reason="perception: STOP")
        elif self._state == "STOPPED_AT_RED" and signal == "GO":
            self._transition("FOLLOWING", reason="perception: explicit GREEN releases brake")

        if self._state == "STOPPED_AT_RED":
            return HighLevelCommand(action="stop", reason="STOPPED_AT_RED")

        # Recovery-escalation memory decays after a stretch of normal driving.
        if self._state != "RECOVERING":
            self._ticks_since_recover += 1
            if self._ticks_since_recover > RECOVER_MEMORY_TICKS:
                self._recover_attempts = 0

        # Stuck guard: front pinned close while we keep commanding motion.
        # Held-None front leaves the count unchanged — a sensor crushed
        # against the wall returns None, which must not erase evidence.
        if self._state != "RECOVERING":
            if held_front is not None:
                if held_front <= STUCK_FRONT_CM:
                    self._stuck_ticks += 1
                else:
                    self._stuck_ticks = 0
            if self._stuck_ticks >= STUCK_TRIGGER_TICKS:
                return self._begin_recovery(
                    left_cm, right_cm,
                    reason=f"stuck: front<={STUCK_FRONT_CM:.0f}cm for "
                           f"{self._stuck_ticks} ticks -> reverse escape",
                )

        # Two-phase reverse escape. RED still preempts via the gate above.
        if self._state == "RECOVERING":
            cmd = self._recover_step()
            if cmd is not None:
                return cmd
            # Recovery finished — fall through and re-evaluate this tick.

        # Init: drive forward until wall detected on either side; rotate
        # first if the front is blocked (never launch into a wall).
        if self._state == "INITIALIZING":
            if (left_cm is not None and left_cm < INIT_WALL_FOUND_CM) or (
                right_cm is not None and right_cm < INIT_WALL_FOUND_CM
            ):
                self._has_found_wall = True
                self._transition("FOLLOWING", reason=f"wall found (l={left_cm}, r={right_cm})")
            elif held_front is not None and held_front < INIT_FRONT_BLOCK_CM:
                return HighLevelCommand(
                    action="pivot_right",
                    linear_speed=INIT_TURN_SPEED,
                    reason="init: front blocked, rotating to find followable wall",
                )
            else:
                return HighLevelCommand(
                    action="forward",
                    linear_speed=INIT_FORWARD_SPEED,
                    reason="searching for wall",
                )

        # Wall-follow controller.
        wf_cmd = self._controller.step(front_cm, left_cm, right_cm)

        # Trigger pivot state if controller demands it; keep its direction.
        if wf_cmd.action.startswith("pivot") and self._state != "PIVOTING":
            self._transition("PIVOTING", reason=wf_cmd.reason)
            self._pivot_ticks = 0
            self._pivot_clear_ticks = 0
            self._pivot_action = wf_cmd.action

        if self._state == "PIVOTING":
            self._pivot_ticks += 1

            # Bailout: spinning too long without clearing -> reverse escape.
            if self._pivot_ticks >= PIVOT_MAX_TICKS:
                return self._begin_recovery(
                    left_cm, right_cm,
                    reason=f"pivot stuck {self._pivot_ticks} ticks -> reverse escape",
                )

            # Front cleared? Judged on the HELD front: a brief None is a
            # dropout (not clear); a sustained None (held is None) means
            # most of a rotation with no echo — treat as open.
            if held_front is None or held_front > PIVOT_EXIT_FRONT_CM:
                self._pivot_clear_ticks += 1
            else:
                self._pivot_clear_ticks = 0

            if self._pivot_clear_ticks >= PIVOT_EXIT_HOLD_TICKS:
                self._transition("FOLLOWING", reason="pivot exit: front cleared")
            else:
                return HighLevelCommand(
                    action=self._pivot_action,  # type: ignore[arg-type]
                    linear_speed=30.0,
                    reason=(
                        f"continuing {self._pivot_action} "
                        f"(ticks {self._pivot_ticks}, clear {self._pivot_clear_ticks})"
                    ),
                )

        return _wf_to_high(wf_cmd)

    # ------------------------------------------------------------------ #
    def _transition(self, new: State, reason: str) -> None:
        old = self._state
        self._state = new
        tracer.state(state=new, from_state=old, reason=reason)

    def _update_held_front(self, front_cm: float | None) -> float | None:
        """Last valid front while a dropout is brief; None once sustained."""
        if front_cm is not None:
            self._last_valid_front = front_cm
            self._front_none_streak = 0
            return front_cm
        self._front_none_streak += 1
        if self._front_none_streak < FRONT_HOLD_TICKS:
            return self._last_valid_front
        return None

    def _begin_recovery(
        self,
        left_cm: float | None,
        right_cm: float | None,
        reason: str,
    ) -> HighLevelCommand:
        self._recover_attempts += 1
        extra = RECOVER_REVERSE_ESCALATION * (self._recover_attempts - 1)
        self._recover_reverse_target = min(
            RECOVER_REVERSE_TICKS_MAX, RECOVER_REVERSE_TICKS + extra
        )
        # Turn toward the more open side after backing out; an unknown
        # (None) side counts as open. Tie keeps the right-hand convention.
        l_v = left_cm if left_cm is not None else float("inf")
        r_v = right_cm if right_cm is not None else float("inf")
        self._recover_dir = "pivot_left" if l_v > r_v else "pivot_right"
        self._recover_ticks = 0
        self._stuck_ticks = 0
        self._ticks_since_recover = 0
        self._transition("RECOVERING", reason)
        return HighLevelCommand(
            action="backward",
            linear_speed=PIVOT_RECOVER_SPEED,
            reason=f"reverse escape start (attempt {self._recover_attempts})",
        )

    def _recover_step(self) -> HighLevelCommand | None:
        """Reverse, then pivot toward open space. None = recovery finished."""
        self._recover_ticks += 1
        if self._recover_ticks <= self._recover_reverse_target:
            return HighLevelCommand(
                action="backward",
                linear_speed=PIVOT_RECOVER_SPEED,
                reason=(
                    f"reverse escape ({self._recover_ticks}/"
                    f"{self._recover_reverse_target})"
                ),
            )
        if self._recover_ticks <= self._recover_reverse_target + RECOVER_TURN_TICKS:
            return HighLevelCommand(
                action=self._recover_dir,
                linear_speed=RECOVER_TURN_SPEED,
                reason=(
                    f"recovery turn {self._recover_dir} "
                    f"({self._recover_ticks - self._recover_reverse_target}/"
                    f"{RECOVER_TURN_TICKS})"
                ),
            )
        self._stuck_ticks = 0
        next_state: State = "FOLLOWING" if self._has_found_wall else "INITIALIZING"
        self._transition(next_state, reason="recovery complete (reverse + turn)")
        return None


def _wf_to_high(cmd: WallFollowCommand) -> HighLevelCommand:
    return HighLevelCommand(
        action=cmd.action,  # type: ignore[arg-type]
        linear_speed=cmd.linear_speed,
        curvature=cmd.curvature,
        reason=cmd.reason,
    )
