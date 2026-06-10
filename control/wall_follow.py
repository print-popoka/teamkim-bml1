"""Smooth wall-following controller — completion-first tuning.

Inputs: filtered distances (cm) from front / left45 / right45 ultrasonics.
Output: ``WallFollowCommand`` with (linear_speed, curvature) — fed into
``Motors.arc(linear, curvature)``.

Field-failure-driven design (2026-06-10 test runs):

  The car must finish the maze ~95/100 runs. Speed is secondary. The
  observed failure modes and their defenses, in priority order:

  F1. U-TURN FAILURE (top killer): the followed right wall ends at a
      baffle/spine end and the car must wrap ~180° around it. A fixed
      open-loop "commit" arc cannot do that. Replaced with a CLOSED-LOOP
      **wall-reacquire turn**: once the right wall is confirmed lost, arc
      right at low speed UNTIL the wall is seen again (or front blocks /
      timeout). See REACQ_* constants.

  F2. ZIGZAG -> side-wall hit: the old junction threshold (40 cm) sat
      inside the band of NORMAL corridor readings (45-deg sensors in a
      ~50 cm corridor read 35-45 cm), so phantom "junction commits"
      fired mid-straight and swerved the car into walls. Openness now
      starts at REACQ_OPEN_CM (60), centering gain is lower, deadband
      wider, centering output capped at CENTERING_MAX, and curvature may
      only GROW by MAX_CURVATURE_STEP per tick.

  F3. LEFT openings are IGNORED (right-hand rule). The old code
      committed into left openings too, which swerved the car off the
      followed wall in turnaround pockets. Left turns happen only via
      front-wall corner anticipation.

  F4. CORNERS missed: corner anticipation starts earlier
      (CORNER_ANTICIPATE_REF_CM raised) with a slower corner floor, the
      open-side choice runs on HELD values so a one-tick side dropout
      cannot flip the turn into a wall that is still there, and an
      unknown front (sustained None) caps speed at APPROACH_SPEED
      instead of letting the car cruise blind.

Noise policy (HC-SR04s sit buried in 3D-printed guides; None bursts are
constant):

  - A side None HOLDS the last valid reading until DEAD_SENSOR_TICKS
    mirroring takes over — a None can never read as "400/open" to the
    PD. Distance continuity: a wall at 15 cm cannot become 400 cm in
    one tick.
  - The wall-reacquire trigger accepts two kinds of "wall lost"
    evidence: sustained VALID far readings (beam crossing the wall edge
    reaches the far side), or a LONG None streak while the front is
    open (real openings at 45 degrees often return no echo at all).
    Both are confirmed over multiple ticks so single dropouts do nothing.
  - The front None holds for FRONT_NONE_HOLD_TICKS, then the default
    flips to open BUT speed is capped at APPROACH_SPEED while unknown.

Actuation smoothing (anti-lurch):

  - Curvature may RELAX toward zero instantly but only GROW away from
    zero by MAX_CURVATURE_STEP per tick (clearance guard and the
    reacquire turn are deliberate maneuvers and bypass this).
  - Speed may drop instantly (braking) but only rise ACCEL_PWM_PER_TICK
    per tick.

Smooth-drive contract (PI feedback) is unchanged: both wheels always
turn while moving; in-place pivots remain dead-end / head-on fallbacks.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Literal

from logs.trace import tracer

SAFE_MARGIN_CM = 4.0
ARC_MIN_CM = 12.0
NARROWING_CM = 14.0

# Head-on emergency: below this front distance an arc cannot clear the
# wall (turn radius ~ half the wheelbase), so we rotate in place toward
# the more open side instead of grinding the nose against the wall.
FRONT_PIVOT_CM = 8.0

# A side sensor returning None for this many CONSECUTIVE ticks is treated as
# DEAD: the controller then mirrors it to the live side so the centering PD
# never steers toward the blind wall. Below this count the last valid
# reading is held — there is deliberately NO gap where None reads as open.
DEAD_SENSOR_TICKS = 8

# Front None holds the last valid reading this many ticks; after that the
# front is UNKNOWN: distance defaults to open for steering purposes, but
# speed is capped at APPROACH_SPEED (never cruise blind into a corner).
FRONT_NONE_HOLD_TICKS = 5

# --------------------------------------------------------------------- #
# Speed table — derived from ONE hardware-day knob, CRUISE.
# Completion-first: CRUISE lowered 38 -> 34 (2026-06-10 field decision:
# 95/100 finishes beat 10 fast finishes).
# --------------------------------------------------------------------- #
CRUISE = 34.0                    # hardware-day knob (straight-corridor speed)
CRUISE_REF = 45.0                # cruise the anticipation table was tuned at
CORNER_APPROACH_FRACTION = 0.70  # corner-approach floor speed = CRUISE * this
SLOW_FRACTION = 2.0 / 3.0        # narrowing/clearance speed   = CRUISE * this

BASE_SPEED = CRUISE                                  # open-straight cruise (importable)
APPROACH_SPEED = CRUISE * CORNER_APPROACH_FRACTION   # corner floor + blind-front cap
SLOW_SPEED = CRUISE * SLOW_FRACTION                  # narrowing + clearance guards

# Centering PD — softened vs the zigzag-era values (KP 0.05/KD 0.06/DB 2.0):
# lower P, more D damping, wider deadband, and a hard cap on how much
# steering the centering term alone may request. Corner anticipation and
# the guards may still saturate steering; lane-keeping may not.
KP_CENTER = 0.035
KD_CENTER = 0.08
DEADBAND_CM = 3.0
CENTERING_MAX = 0.5

KP_CORNER = 0.05

# Corner anticipation is SPEED-AWARE: lead distance is
# CORNER_ANTICIPATE_REF_CM at CRUISE_REF, +/- ANTICIPATE_GAIN cm per unit
# of CRUISE away from the reference, floored at MIN_ANTICIPATE_CM.
CORNER_ANTICIPATE_REF_CM = 45.0
MIN_ANTICIPATE_CM = 20.0
ANTICIPATE_GAIN = 0.5

# front_speed ramp: corner-floor plateau below CORNER_FLOOR_CM, linear
# ramp up to full CRUISE at the anticipation onset.
CORNER_FLOOR_CM = 12.0

# Cap on |derror| per tick (D-term spike guard on sensor jumps).
MAX_DERROR_CM = 5.0

# As |error| grows, scale down speed (don't overshoot corrections).
SPEED_SCALE_ERROR_REF = 20.0
SPEED_SCALE_FLOOR = 0.55

# Wide-area cap: both sides beyond this means no usable lateral reference
# (open pocket / turnaround) — hold below full cruise.
WIDE_OPEN_CM = 60.0
WIDE_AREA_SPEED_FRACTION = 0.85

# Anti-lurch slew limits.
MAX_CURVATURE_STEP = 0.25     # max growth of |curvature| per tick (PD path)
ACCEL_PWM_PER_TICK = 2.0      # max speed increase per tick; decel is instant

# --------------------------------------------------------------------- #
# Wall-reacquire turn (the U-turn fix, F1) — closed-loop right turn that
# runs when the followed right wall is confirmed lost.
#
#   ARM:      right shows a valid wall (<= WALL_PRESENT_CM) for
#             REACQ_ARM_TICKS consecutive ticks. Arming survives until a
#             reacquire actually triggers.
#   TRIGGER:  armed AND (valid right > REACQ_OPEN_CM for
#             REACQ_CONFIRM_TICKS ticks, OR right None for
#             REACQ_NONE_TICKS ticks while the front is not blocked).
#   DO:       drive STRAIGHT for REACQ_STRAIGHT_TICKS (the 45-deg sensor
#             spots the opening ~0.7 x wall-distance BEFORE the car body
#             reaches the wall end — turning immediately clips the end),
#             then arc(REACQ_SPEED, REACQ_CURVATURE) until the wall is
#             reacquired.
#   FINISH:   right shows a valid wall (<= WALL_PRESENT_CM) for
#             REACQ_SUCCESS_TICKS ticks (wall reacquired -> resume PD),
#             OR held front < REACQ_FRONT_ABORT_CM (corner logic owns it),
#             OR REACQ_MAX_TICKS timeout.
#
# A phantom trigger (None burst while the wall is still there) is cheap:
# the straight phase changes nothing, valid readings return as the beam
# geometry shifts, and the success condition ends the maneuver within a
# few ticks — a bounded low-speed hiccup, not a crash.
# --------------------------------------------------------------------- #
WALL_PRESENT_CM = 45.0
REACQ_ARM_TICKS = 3
REACQ_OPEN_CM = 60.0
REACQ_CONFIRM_TICKS = 2
REACQ_NONE_TICKS = 6
REACQ_FRONT_ABORT_CM = 22.0
REACQ_STRAIGHT_TICKS = 18     # pass the wall end before starting the wrap
                              # (~0.7 x corridor wall distance at REACQ_SPEED)
REACQ_MAX_TICKS = 110         # total (straight + turn) budget; the closed
                              # loop normally exits long before this
REACQ_SUCCESS_TICKS = 2
REACQ_CURVATURE = -0.5        # wide wrap; tight pockets are handed to the
                              # corner logic by the front-abort instead
REACQ_SPEED_FRACTION = 0.75
REACQ_SPEED = CRUISE * REACQ_SPEED_FRACTION


def corner_anticipate_cm(cruise: float = CRUISE) -> float:
    """Front distance at which cornering begins, as a function of cruise."""
    lead = CORNER_ANTICIPATE_REF_CM + ANTICIPATE_GAIN * (cruise - CRUISE_REF)
    return max(MIN_ANTICIPATE_CM, lead)


def front_speed(front_cm: float, cruise: float = CRUISE) -> float:
    """Continuous forward-clearance speed profile (FRONT sensor only)."""
    onset = corner_anticipate_cm(cruise)
    floor = cruise * CORNER_APPROACH_FRACTION
    if front_cm >= onset:
        return cruise
    if front_cm <= CORNER_FLOOR_CM:
        return floor
    t = (front_cm - CORNER_FLOOR_CM) / (onset - CORNER_FLOOR_CM)
    return floor + t * (cruise - floor)


# Module alias used by the controller at runtime (cruise == CRUISE here).
CORNER_ANTICIPATE_CM = corner_anticipate_cm(CRUISE)


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
        # Consecutive-None counters, for hold filters + dead-sensor detection.
        self._left_none_streak: int = 0
        self._right_none_streak: int = 0
        self._front_none_streak: int = 0
        # Hold filters: last valid reading per sensor.
        self._last_valid_left: float | None = None
        self._last_valid_right: float | None = None
        self._last_valid_front: float | None = None
        # Wall-reacquire (U-turn) state.
        self._right_wall_streak: int = 0
        self._right_valid_open_streak: int = 0
        self._reacq_armed: bool = False
        self._reacq_active: bool = False
        self._reacq_ticks: int = 0
        self._reacq_success_streak: int = 0
        # Actuation slew state. Curvature starts from "straight" so the
        # very first command cannot be a full-lock lurch; speed starts
        # unconstrained (None) so one-shot uses see the true profile.
        self._last_curvature: float = 0.0
        self._last_speed: float | None = None

    @property
    def in_reacquire(self) -> bool:
        """True while the closed-loop right-wall reacquire turn runs."""
        return self._reacq_active

    def step(
        self,
        front_cm: float | None,
        left_cm: float | None,
        right_cm: float | None,
    ) -> WallFollowCommand:
        # --- None streaks + hold filters --------------------------------
        if left_cm is not None:
            self._last_valid_left = left_cm
            self._left_none_streak = 0
        else:
            self._left_none_streak += 1

        if right_cm is not None:
            self._last_valid_right = right_cm
            self._right_none_streak = 0
        else:
            self._right_none_streak += 1

        if front_cm is not None:
            self._last_valid_front = front_cm
            self._front_none_streak = 0
        else:
            self._front_none_streak += 1

        l_filtered = left_cm
        if (
            l_filtered is None
            and self._left_none_streak < DEAD_SENSOR_TICKS
            and self._last_valid_left is not None
        ):
            l_filtered = self._last_valid_left

        r_filtered = right_cm
        if (
            r_filtered is None
            and self._right_none_streak < DEAD_SENSOR_TICKS
            and self._last_valid_right is not None
        ):
            r_filtered = self._last_valid_right

        f_filtered = front_cm
        front_unknown = False
        if f_filtered is None:
            if (
                self._front_none_streak < FRONT_NONE_HOLD_TICKS
                and self._last_valid_front is not None
            ):
                f_filtered = self._last_valid_front
            else:
                front_unknown = True

        f = self._safe(f_filtered, default=400.0)
        l = self._safe(l_filtered, default=400.0)
        r = self._safe(r_filtered, default=400.0)

        # Dead-side mirroring (sustained dropout): never steer toward the
        # blind wall. Both dead -> both 400 -> straight.
        left_dead = self._left_none_streak >= DEAD_SENSOR_TICKS
        right_dead = self._right_none_streak >= DEAD_SENSOR_TICKS
        if left_dead and not right_dead:
            l = r
        elif right_dead and not left_dead:
            r = l

        # --- Wall-presence bookkeeping for the reacquire turn ------------
        right_wall_now = right_cm is not None and right_cm <= WALL_PRESENT_CM
        if right_wall_now:
            self._right_wall_streak += 1
            if self._right_wall_streak >= REACQ_ARM_TICKS:
                self._reacq_armed = True
        else:
            self._right_wall_streak = 0
        if right_cm is not None and right_cm > REACQ_OPEN_CM:
            self._right_valid_open_streak += 1
        else:
            self._right_valid_open_streak = 0

        # --- Pivot fallbacks (preempt everything; cancel reacquire) ------
        if f < ARC_MIN_CM and l < ARC_MIN_CM and r < ARC_MIN_CM:
            self._end_reacquire("dead-end")
            action: Action = "pivot_left" if l > r else "pivot_right"
            return self._emit(WallFollowCommand(
                action=action,
                linear_speed=30.0,
                reason=f"dead-end (f={f:.1f} l={l:.1f} r={r:.1f})",
            ))

        if f < FRONT_PIVOT_CM:
            self._end_reacquire("front emergency")
            action = "pivot_left" if l > r else "pivot_right"
            return self._emit(WallFollowCommand(
                action=action,
                linear_speed=30.0,
                reason=f"front emergency (f={f:.1f} l={l:.1f} r={r:.1f})",
            ))

        # --- Clearance guards --------------------------------------------
        if l < SAFE_MARGIN_CM:
            return self._emit(WallFollowCommand(
                action="arc",
                linear_speed=SLOW_SPEED,
                curvature=-0.8,
                reason=f"clearance left ({l:.1f}<{SAFE_MARGIN_CM:.1f})",
            ))
        if r < SAFE_MARGIN_CM:
            # Wall is back (very close); the reacquire turn is over.
            self._end_reacquire("clearance right")
            return self._emit(WallFollowCommand(
                action="arc",
                linear_speed=SLOW_SPEED,
                curvature=+0.8,
                reason=f"clearance right ({r:.1f}<{SAFE_MARGIN_CM:.1f})",
            ))

        # --- Wall-reacquire turn (F1: the U-turn around a wall end) ------
        if self._reacq_active:
            self._reacq_ticks += 1
            if right_wall_now:
                self._reacq_success_streak += 1
            else:
                self._reacq_success_streak = 0

            if self._reacq_success_streak >= REACQ_SUCCESS_TICKS:
                self._end_reacquire("wall reacquired")
            elif f < REACQ_FRONT_ABORT_CM:
                self._end_reacquire("front blocked, corner logic takes over")
            elif self._reacq_ticks > REACQ_MAX_TICKS:
                self._end_reacquire("timeout")
            else:
                in_straight = self._reacq_ticks <= REACQ_STRAIGHT_TICKS
                phase = "straight" if in_straight else "turn"
                return self._emit(WallFollowCommand(
                    action="arc",
                    linear_speed=REACQ_SPEED,
                    curvature=0.0 if in_straight else REACQ_CURVATURE,
                    reason=(
                        f"reacquire_right ({phase} "
                        f"t={self._reacq_ticks}/{REACQ_MAX_TICKS}) "
                        f"f={f:.1f} l={l:.1f} r={r:.1f}"
                    ),
                ))
        elif self._reacq_armed:
            valid_open = self._right_valid_open_streak >= REACQ_CONFIRM_TICKS
            none_open = (
                self._right_none_streak >= REACQ_NONE_TICKS
                and f > REACQ_FRONT_ABORT_CM
            )
            if valid_open or none_open:
                self._reacq_armed = False
                self._reacq_active = True
                self._reacq_ticks = 1
                self._reacq_success_streak = 0
                tracer.info(
                    "right_wall_lost_reacquire",
                    front=f, left=l, right=r,
                    trigger="valid_far" if valid_open else "none_streak",
                )
                return self._emit(WallFollowCommand(
                    action="arc",
                    linear_speed=REACQ_SPEED,
                    curvature=0.0,  # straight phase first: pass the wall end
                    reason=f"reacquire_right (start) f={f:.1f} l={l:.1f} r={r:.1f}",
                ))

        # --- Smooth drive (centering PD + corner anticipation) -----------
        error = r - l
        derror_raw = 0.0 if self._last_error is None else (error - self._last_error)
        derror = max(-MAX_DERROR_CM, min(MAX_DERROR_CM, derror_raw))
        self._last_error = error

        if abs(error) < DEADBAND_CM:
            centering = 0.0
        else:
            centering = -KP_CENTER * error - KD_CENTER * derror
            # Lane-keeping alone may never saturate steering (zigzag fix).
            centering = max(-CENTERING_MAX, min(CENTERING_MAX, centering))

        corner_bias = 0.0
        if f < CORNER_ANTICIPATE_CM:
            shortfall = CORNER_ANTICIPATE_CM - f
            # Open-side choice on the HELD values: a brief dropout on one
            # side must not flip the turn toward a wall that is still
            # there (the hold filter encodes distance continuity, so the
            # held reading IS our best estimate of that side).
            direction = -1.0 if r >= l else +1.0
            corner_bias = direction * KP_CORNER * shortfall

        curvature_target = max(-1.0, min(1.0, centering + corner_bias))
        curvature = self._slew_curvature(curvature_target)

        speed = front_speed(f)
        if l < NARROWING_CM and r < NARROWING_CM:
            speed = min(speed, SLOW_SPEED)
        if l > WIDE_OPEN_CM and r > WIDE_OPEN_CM:
            speed = min(speed, CRUISE * WIDE_AREA_SPEED_FRACTION)
        if front_unknown:
            # Blind front: do not cruise into what we cannot see.
            speed = min(speed, APPROACH_SPEED)

        scale = max(
            SPEED_SCALE_FLOOR,
            1.0 - abs(error) / SPEED_SCALE_ERROR_REF * (1.0 - SPEED_SCALE_FLOOR),
        )
        speed = self._slew_speed(speed * scale)

        return self._emit(WallFollowCommand(
            action="arc",
            linear_speed=speed,
            curvature=curvature,
            reason=(
                f"f={f:.1f} l={l:.1f} r={r:.1f} "
                f"err={error:+.1f} de={derror:+.1f} "
                f"cent={centering:+.2f} corn={corner_bias:+.2f} "
                f"scale={scale:.2f}"
            ),
        ))

    # ------------------------------------------------------------------ #
    # Internals
    # ------------------------------------------------------------------ #
    def _end_reacquire(self, why: str) -> None:
        if self._reacq_active:
            tracer.info("reacquire_done", why=why, ticks=self._reacq_ticks)
        self._reacq_active = False
        self._reacq_ticks = 0
        self._reacq_success_streak = 0

    def _slew_curvature(self, target: float) -> float:
        """Limit how fast |curvature| may GROW; relaxing toward zero is
        instant (returning to straight is never dangerous — jerking into
        a turn on one noisy tick is)."""
        last = self._last_curvature
        shrinking_same_side = abs(target) <= abs(last) and target * last >= 0
        if shrinking_same_side:
            return target
        delta = max(-MAX_CURVATURE_STEP, min(MAX_CURVATURE_STEP, target - last))
        return last + delta

    def _slew_speed(self, target: float) -> float:
        """Brake instantly, accelerate gradually."""
        last = self._last_speed
        if last is None or target <= last:
            return target
        return min(target, last + ACCEL_PWM_PER_TICK)

    def _emit(self, cmd: WallFollowCommand) -> WallFollowCommand:
        # Record actuation state for the slew limiters. A pivot resets the
        # steering reference (rotation in place has no arc curvature).
        self._last_speed = cmd.linear_speed
        self._last_curvature = cmd.curvature if cmd.action == "arc" else 0.0
        tracer.decision(
            state="WALL_FOLLOW",
            action=f"{cmd.action} s={cmd.linear_speed:.0f} c={cmd.curvature:+.2f}",
            reason=cmd.reason,
        )
        return cmd

    @staticmethod
    def _safe(v: float | None, default: float) -> float:
        return v if v is not None else default
