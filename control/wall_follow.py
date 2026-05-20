"""Smooth wall-following controller.

Inputs: filtered distances (cm) from front / left45 / right45 ultrasonics.
Output: ``WallFollowCommand`` with (linear_speed, curvature) — fed into
``Motors.arc(linear, curvature)``.

Design (per CLAUDE.md, PI feedback "smooth, never stop-and-turn"):

  1. **Center-following PD** in straight corridors.
     error = right_dist - left_dist (positive => more room on right =>
     curve right, i.e. negative curvature in our convention).

  2. **Corner anticipation from front distance**. As front shortens, we
     start curving toward the side with more space — well before we'd
     hit the wall. Produces a smooth arc through 90 deg corners.

  3. **Narrowing detection -> slow down**. If both side distances drop
     together (corridor narrowing), reduce linear speed so the PD has
     more time to correct.

  4. **Clearance guard**. If any side distance falls below SAFE_MARGIN_CM,
     force a curve away from that wall regardless of normal PD.

  5. **Pivot fallback**. If front is so close that an arc can't fit
     (sub ARC_MIN_CM), raise ``need_pivot`` so the state machine can call
     ``Motors.pivot_*``. Dead-end U-turn path only.

Tuning constants are PLACEHOLDERS; refine after sample-maze test.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Literal

from logs.trace import tracer  # noqa: F401 — used by _maybe_log_junction + _trace

SAFE_MARGIN_CM = 4.0
CORNER_ANTICIPATE_CM = 25.0
ARC_MIN_CM = 12.0
NARROWING_CM = 14.0

BASE_SPEED = 45.0
SLOW_SPEED = 30.0
APPROACH_SPEED = 35.0

KP_CENTER = 0.06
KP_CORNER = 0.03
KD_CENTER = 0.04

# Errors below this (cm) get no centering correction.
# Addresses prof's tip #2: teams over-correct and waste forward progress.
DEADBAND_CM = 1.0

# Cap on |derror| per tick. Without this, a side wall disappearing at
# a junction makes derror huge (e.g. 15 -> 400 in one tick = 385cm),
# KD * derror dominates and the controller spasms for one tick.
MAX_DERROR_CM = 5.0

# When right_cm exceeds this, the right wall has effectively disappeared
# (right-opening / T-junction / cross intersection). On the rising edge
# we COMMIT to a sharp right curve for JUNCTION_COMMIT_TICKS, ignoring
# what the PD says. This stops the car from "smoothing through" a tight
# right opening and missing the turn.
JUNCTION_CM = 40.0
JUNCTION_COMMIT_TICKS = 6           # 6 ticks @ 10Hz = 600ms of committed arc
JUNCTION_COMMIT_CURVATURE = -0.85   # sharp right (not max -1.0 — keep some smoothness)
JUNCTION_COMMIT_SPEED = 35.0        # slower during the commit for safety

# As |error| grows, scale down speed. Big errors mean we are off-center
# (or in a junction); driving slower while we correct keeps the car from
# overshooting and reduces wall scrapes. Linear ramp; bottoms out at
# SPEED_SCALE_FLOOR so we never freeze.
SPEED_SCALE_ERROR_REF = 20.0  # error in cm at which speed drops to floor
SPEED_SCALE_FLOOR = 0.55      # never below 55% of requested speed


Action = Literal["arc", "pivot_right", "pivot_left", "stop"]


@dataclass(frozen=True)
class WallFollowCommand:
    action: Action
    linear_speed: float = 0.0
    curvature: float = 0.0
    reason: str = ""


class WallFollowController:
    """Stateful smooth wall-follow controller. Right-hand bias on ties."""

    def __init__(self) -> None:
        self._last_error: float | None = None
        # ``None`` means "we haven't seen this side yet". On the first tick
        # the junction-edge detector seeds itself with the current state
        # instead of treating it as a transition — otherwise booting in
        # an open area would trigger a phantom right-turn commit.
        self._right_open_active: bool | None = None
        self._left_open_active: bool | None = None
        self._commit_ticks_remaining: int = 0
        self._commit_curvature: float = 0.0

    def step(
        self,
        front_cm: float | None,
        left_cm: float | None,
        right_cm: float | None,
    ) -> WallFollowCommand:
        f = self._safe(front_cm, default=400.0)
        l = self._safe(left_cm, default=400.0)
        r = self._safe(right_cm, default=400.0)

        # 5. Pivot fallback ---------------------------------------------
        if f < ARC_MIN_CM and l < ARC_MIN_CM and r < ARC_MIN_CM:
            cmd = WallFollowCommand(
                action="pivot_right",
                linear_speed=30.0,
                reason=f"dead-end (f={f:.1f} l={l:.1f} r={r:.1f})",
            )
            self._trace(cmd)
            return cmd

        # 4. Clearance guard --------------------------------------------
        if l < SAFE_MARGIN_CM:
            cmd = WallFollowCommand(
                action="arc",
                linear_speed=SLOW_SPEED,
                curvature=-0.8,
                reason=f"clearance left ({l:.1f}<{SAFE_MARGIN_CM:.1f})",
            )
            self._trace(cmd)
            return cmd
        if r < SAFE_MARGIN_CM:
            cmd = WallFollowCommand(
                action="arc",
                linear_speed=SLOW_SPEED,
                curvature=+0.8,
                reason=f"clearance right ({r:.1f}<{SAFE_MARGIN_CM:.1f})",
            )
            self._trace(cmd)
            return cmd

        # Junction detection — emits trace + sets up commit on rising edge.
        new_right_open, new_left_open = self._maybe_log_junction(f, l, r)
        # First tick: just seed; never treat boot state as an "edge".
        if self._right_open_active is None or self._left_open_active is None:
            self._right_open_active = new_right_open
            self._left_open_active = new_left_open
        else:
            if new_right_open and not self._right_open_active:
                self._commit_ticks_remaining = JUNCTION_COMMIT_TICKS
                self._commit_curvature = JUNCTION_COMMIT_CURVATURE  # right
            elif new_left_open and not self._left_open_active:
                self._commit_ticks_remaining = JUNCTION_COMMIT_TICKS
                self._commit_curvature = -JUNCTION_COMMIT_CURVATURE  # left
            self._right_open_active = new_right_open
            self._left_open_active = new_left_open

        # Junction commit overrides normal PD for N ticks after detecting
        # a side opening. Keeps the car committed to the turn even if the
        # mid-rotation sensor readings get weird.
        if self._commit_ticks_remaining > 0:
            self._commit_ticks_remaining -= 1
            # Still record error for the D term continuity.
            error_commit = r - l
            self._last_error = error_commit
            cmd = WallFollowCommand(
                action="arc",
                linear_speed=JUNCTION_COMMIT_SPEED,
                curvature=self._commit_curvature,
                reason=f"junction_commit (left={self._commit_ticks_remaining}) "
                       f"f={f:.1f} l={l:.1f} r={r:.1f}",
            )
            self._trace(cmd)
            return cmd

        # 1+2+3. Smooth drive -------------------------------------------
        error = r - l
        derror_raw = 0.0 if self._last_error is None else (error - self._last_error)
        # Clamp D-term to prevent spasms on big sensor jumps.
        derror = max(-MAX_DERROR_CM, min(MAX_DERROR_CM, derror_raw))
        self._last_error = error

        if abs(error) < DEADBAND_CM:
            centering = 0.0
        else:
            centering = -KP_CENTER * error - KD_CENTER * derror

        corner_bias = 0.0
        if f < CORNER_ANTICIPATE_CM:
            shortfall = CORNER_ANTICIPATE_CM - f
            direction = -1.0 if r >= l else +1.0
            corner_bias = direction * KP_CORNER * shortfall

        curvature = max(-1.0, min(1.0, centering + corner_bias))

        # Base speed selection: narrowing > corner approach > normal.
        if l < NARROWING_CM and r < NARROWING_CM:
            speed = SLOW_SPEED
        elif f < CORNER_ANTICIPATE_CM:
            speed = APPROACH_SPEED
        else:
            speed = BASE_SPEED

        # Error-magnitude speed scaling: big offset -> slow down so we
        # don't overshoot while correcting. Linear ramp to a floor.
        scale = max(
            SPEED_SCALE_FLOOR,
            1.0 - abs(error) / SPEED_SCALE_ERROR_REF * (1.0 - SPEED_SCALE_FLOOR),
        )
        speed = speed * scale

        cmd = WallFollowCommand(
            action="arc",
            linear_speed=speed,
            curvature=curvature,
            reason=(
                f"f={f:.1f} l={l:.1f} r={r:.1f} "
                f"err={error:+.1f} de={derror:+.1f} "
                f"cent={centering:+.2f} corn={corner_bias:+.2f} "
                f"scale={scale:.2f}"
            ),
        )
        self._trace(cmd)
        return cmd

    # ------------------------------------------------------------------ #
    # Internals
    # ------------------------------------------------------------------ #
    def _maybe_log_junction(
        self, f: float, l: float, r: float
    ) -> tuple[bool, bool]:
        """Detect openings. Emit a trace event on rising edges.

        Returns (right_open_now, left_open_now); caller compares against
        previous state to detect a rising edge.
        """
        right_open = r > JUNCTION_CM
        left_open = l > JUNCTION_CM
        # Don't log on the first call (active flags are None) — that's
        # initial state, not an edge.
        if self._right_open_active is True or self._right_open_active is False:
            if right_open and not self._right_open_active:
                tracer.info("junction_right_opened", front=f, left=l, right=r)
        if self._left_open_active is True or self._left_open_active is False:
            if left_open and not self._left_open_active:
                tracer.info("junction_left_opened", front=f, left=l, right=r)
        return right_open, left_open

    @staticmethod
    def _safe(v: float | None, default: float) -> float:
        return v if v is not None else default

    @staticmethod
    def _trace(cmd: WallFollowCommand) -> None:
        tracer.decision(
            state="WALL_FOLLOW",
            action=f"{cmd.action} s={cmd.linear_speed:.0f} c={cmd.curvature:+.2f}",
            reason=cmd.reason,
        )
