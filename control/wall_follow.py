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

from logs.trace import tracer

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

        # 1+2+3. Smooth drive -------------------------------------------
        error = r - l
        derror = 0.0 if self._last_error is None else (error - self._last_error)
        self._last_error = error
        centering = -KP_CENTER * error - KD_CENTER * derror

        corner_bias = 0.0
        if f < CORNER_ANTICIPATE_CM:
            shortfall = CORNER_ANTICIPATE_CM - f
            direction = -1.0 if r >= l else +1.0
            corner_bias = direction * KP_CORNER * shortfall

        curvature = max(-1.0, min(1.0, centering + corner_bias))

        if l < NARROWING_CM and r < NARROWING_CM:
            speed = SLOW_SPEED
        elif f < CORNER_ANTICIPATE_CM:
            speed = APPROACH_SPEED
        else:
            speed = BASE_SPEED

        cmd = WallFollowCommand(
            action="arc",
            linear_speed=speed,
            curvature=curvature,
            reason=(
                f"f={f:.1f} l={l:.1f} r={r:.1f} "
                f"err={error:+.1f} de={derror:+.1f} "
                f"cent={centering:+.2f} corn={corner_bias:+.2f}"
            ),
        )
        self._trace(cmd)
        return cmd

    # ------------------------------------------------------------------ #
    # Internals
    # ------------------------------------------------------------------ #
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
