"""Guided 3-sensor ultrasonic integration + DIRECTION check.

Verifies two things the plain ``hardware_check.py`` cannot:

  1. **Integration** — the production ``hal.ultrasonics.Ultrasonics`` manager
     returns usable filtered distances for all three sensors at once
     (``poll_all``). If the 3-sensor wiring/integration is broken, this fails.

  2. **Direction / placement** — that the sensor *labelled* "left" actually
     faces left, etc. ``hardware_check.py`` tests one sensor at a time by its
     label and so CANNOT catch a left<->right swap (a swapped pair still
     "reads a distance"). This tool puts an obstacle on ONE side and checks
     that the sensor we EXPECT is the one that actually sees it. A swap, a
     dead sensor, or a mis-aimed sensor all show up as a clear FAIL.

The operator is NOT the software author — every prompt and verdict is
self-explanatory, in Korean, with the exact physical action to take.

Run on the Raspberry Pi (wheels may stay on — sensor-only, no motor):

    cd ~/teamkim-bml1
    python ultrasonic_direction_check.py

Preview the whole flow OFF the Pi (no hardware, scripted readings):

    python ultrasonic_direction_check.py --demo

Pin map (BCM, from CLAUDE.md): FRONT 23/24, LEFT45 25/8, RIGHT45 7/12.
The check reads the live pin map from ``hal.ultrasonics`` so it can never
drift from what the robot actually uses.
"""

from __future__ import annotations

import argparse
import sys
import time
from dataclasses import dataclass

# Same off-Pi-importable pattern as hal/*. Only the hardware flow touches
# GPIO; the pure decision logic (classify_direction) imports anywhere.
try:
    import RPi.GPIO as GPIO  # type: ignore[import-not-found]
except Exception:  # noqa: BLE001
    GPIO = None  # type: ignore[assignment]

from hal.ultrasonics import DEFAULT_PINS, SENSOR_MOUNTS, Ultrasonics
from logs.trace import tracer

# --------------------------------------------------------------------- #
# Thresholds (cm) — generous on purpose; this is a yes/no direction check,
# not a calibration. The operator places the object CLOSE to the target
# sensor and keeps the other two clear.
# --------------------------------------------------------------------- #
NEAR_CM = 25.0   # the target sensor must read within this when the object is placed
FAR_CM = 40.0    # the other sensors must stay beyond this (kept clear)
SAMPLES = 6      # poll_all rounds per step (fills the hal median window)

# Human direction -> the sensor key it must light up (hal uses left45/right45).
EXPECTED_SENSOR = {"front": "front", "left": "left45", "right": "right45"}
DIRECTION_ORDER = ("front", "left", "right")

# Pin numbers come from hal's LIVE pin map (DEFAULT_PINS) so the displayed
# wiring can never drift from what the robot actually drives.
_KO_NAME = {"front": "앞 FRONT", "left45": "왼쪽 LEFT45", "right45": "오른쪽 RIGHT45"}
LABEL = {
    key: f"{_KO_NAME.get(key, key)} ({trig}/{echo})"
    for key, (trig, echo) in DEFAULT_PINS.items()
}
PLACEMENT = {
    "front": (
        "FRONT는 차 앞쪽(사진 기준 위쪽) 중앙 노즈에 정면 0°로 장착. "
        "차 정면 바로 앞 10~15cm 에 평평한 판/손바닥을 대세요."
    ),
    "left": (
        "LEFT45는 앞-왼쪽 모서리에 전방 왼쪽 45°로 장착. "
        "그 센서가 향하는 방향 10~15cm 에 대세요."
    ),
    "right": (
        "RIGHT45는 앞-오른쪽 모서리에 전방 오른쪽 45°로 장착. "
        "그 센서가 향하는 방향 10~15cm 에 대세요."
    ),
}

PASS = "PASS"
WRONG_SENSOR = "WRONG_SENSOR"
DEAD_EXPECTED = "DEAD_EXPECTED"
NO_DETECTION = "NO_DETECTION"
AMBIGUOUS = "AMBIGUOUS"


@dataclass(frozen=True)
class DirectionVerdict:
    direction: str                       # "front" | "left" | "right"
    expected: str                        # the hal sensor key that should see it
    readings: dict[str, float | None]    # all sensor distances this step
    nearest: str | None                  # which sensor actually read nearest
    status: str                          # PASS / WRONG_SENSOR / DEAD_EXPECTED / ...
    detail: str                          # operator-facing Korean explanation


def classify_direction(
    direction: str,
    readings: dict[str, float | None],
    near_cm: float = NEAR_CM,
    far_cm: float = FAR_CM,
) -> DirectionVerdict:
    """Pure decision: did the EXPECTED sensor see the obstacle, and only it?

    No GPIO, no I/O — unit-testable. ``readings`` maps every hal sensor key
    to a filtered distance in cm (or None for no echo / dead sensor).
    """
    expected = EXPECTED_SENSOR[direction]
    expected_val = readings.get(expected)
    valid = {k: v for k, v in readings.items() if v is not None}
    nearest = min(valid, key=lambda k: valid[k]) if valid else None
    # The expected sensor returned nothing at all -> dead / miswired.
    if expected_val is None:
        return DirectionVerdict(
            direction, expected, readings, nearest, DEAD_EXPECTED,
            f"기대 센서 {LABEL[expected]} 가 신호 없음(None). 죽었거나 배선 문제 "
            f"— docs/hardware_troubleshooting.md 참고.",
        )

    # Did the EXPECTED sensor itself see a near object? If so, this direction
    # is wired/aimed correctly — it is NEVER a swap, even if a neighbour ties
    # or reads marginally closer. Warn only if a neighbour is also close
    # (object too central / cross-talk); otherwise a clean PASS.
    if expected_val <= near_cm:
        intruders = {
            k: v for k, v in readings.items()
            if k != expected and v is not None and v < far_cm
        }
        if intruders:
            names = ", ".join(f"{LABEL[k]}={v:.0f}cm" for k, v in intruders.items())
            return DirectionVerdict(
                direction, expected, readings, nearest, AMBIGUOUS,
                f"{LABEL[expected]} 가 {expected_val:.0f}cm 로 잡혔지만 다른 센서도 "
                f"가까움({names}). 장애물을 더 한쪽으로 치우쳐 다시 시도.",
            )
        return DirectionVerdict(
            direction, expected, readings, nearest, PASS,
            f"{LABEL[expected]} 가 {expected_val:.0f}cm 로 단독 감지. 방향 정확.",
        )

    # Expected sensor did NOT see a near object. If ANOTHER sensor did, the
    # wrong sensor is facing this direction -> swap / mis-aim.
    near_others = {
        k: v for k, v in valid.items() if k != expected and v <= near_cm
    }
    if near_others:
        culprit = min(near_others, key=lambda k: near_others[k])
        swap = {"left45": "right45", "right45": "left45"}.get(expected) == culprit
        hint = " (왼쪽↔오른쪽 배선/장착이 바뀐 것으로 보임)" if swap else ""
        return DirectionVerdict(
            direction, expected, readings, culprit, WRONG_SENSOR,
            f"{direction.upper()} 방향에 댔는데 {LABEL[culprit]} 가 "
            f"{near_others[culprit]:.0f}cm 로 잡힘{hint}. 기대={LABEL[expected]}.",
        )

    # Nobody saw a near object -> not placed / too far.
    return DirectionVerdict(
        direction, expected, readings, nearest, NO_DETECTION,
        f"가깝게(≤{near_cm:.0f}cm) 잡힌 센서가 없음 (기대 {LABEL[expected]}="
        f"{expected_val:.0f}cm). 장애물을 해당 센서 앞 10~15cm 에 두고 다시 시도.",
    )


# --------------------------------------------------------------------- #
# Hardware-side flow (Pi only)
# --------------------------------------------------------------------- #
def require_gpio() -> None:
    if GPIO is None:
        print("[FAIL] RPi.GPIO 가 없습니다. 이 스크립트는 라즈베리파이에서 실행하세요.")
        print("       (Pi 없이 흐름만 보려면:  python ultrasonic_direction_check.py --demo)")
        sys.exit(1)


def _fmt(readings: dict[str, float | None]) -> str:
    return "  ".join(
        f"{k}={'--' if v is None else f'{v:.0f}cm'}" for k, v in readings.items()
    )


def poll_stable(us: Ultrasonics, samples: int = SAMPLES) -> dict[str, float | None]:
    """Poll all sensors a few rounds so the hal median window fills, then
    return the last (filtered) snapshot."""
    last: dict[str, float | None] = {}
    for _ in range(max(1, samples)):
        last = us.poll_all()
        time.sleep(0.02)
    return last


def print_guide() -> None:
    print("=" * 64)
    print(" 초음파 3센서 통합 + 방향 배치 점검")
    print("=" * 64)
    print(" 목적: ① hal 3센서 통합이 도는지  ② 앞/왼/오 센서가 실제로")
    print("       그 방향을 향하게 배선·장착됐는지(좌우 바뀜까지) 확인.")
    print()
    print(" 준비물: 평평한 판(책/손바닥). 모터/바퀴는 안 써도 됩니다.")
    print(" 방법: 각 단계에서 '한 방향'에만 장애물을 10~15cm 대고,")
    print("       나머지 방향은 50cm 이상 비워둔 뒤 Enter.")
    print("       기대한 센서가 단독으로 가깝게 잡히면 PASS.")
    print(" 장착: FRONT=앞 중앙 0°, LEFT45=앞-왼쪽 45°, RIGHT45=앞-오른쪽 45°.")
    print("       세 센서는 같은 높이/수평, 차체나 전선에 초음파 원뿔이 막히지 않게.")
    print("=" * 64)
    print()


def print_step_guide(direction: str) -> None:
    """Per-direction operator guidance (the ``[FRONT]`` header + where to
    place the object). Shared by the live run and ``--demo`` so the two
    show the SAME guide and can't drift."""
    sensor = EXPECTED_SENSOR[direction]
    mount = SENSOR_MOUNTS[sensor]
    print(f"[{direction.upper()}] {LABEL[sensor]} 점검")
    print(f"    장착 기준: {mount.position}, yaw={mount.yaw_deg:+d}°")
    print(f"    {PLACEMENT[direction]}  (다른 방향은 비워두세요) → Enter")


def print_verdict(v: DirectionVerdict) -> None:
    mark = {PASS: "✅ PASS", AMBIGUOUS: "⚠️  WARN"}.get(v.status, "❌ FAIL")
    print(f"  [{mark}] {v.direction.upper()}  ({_fmt(v.readings)})")
    print(f"         {v.detail}")
    tracer.info(
        "direction_check",
        direction=v.direction,
        expected=v.expected,
        nearest=v.nearest,
        status=v.status,
    )


def print_summary(results: list[DirectionVerdict]) -> None:
    print()
    print("=" * 64)
    print(" 요약")
    print("=" * 64)
    for v in results:
        print(f"  {v.direction.upper():5} : {v.status}")

    all_pass = all(v.status == PASS for v in results)
    swaps = [v for v in results if v.status == WRONG_SENSOR]
    dead = [v for v in results if v.status == DEAD_EXPECTED]
    retry = [v for v in results if v.status in (NO_DETECTION, AMBIGUOUS)]

    print()
    if all_pass:
        print("  ✅ 통합 OK + 앞/왼/오 방향 배치 모두 정확. main.py 주행 준비 완료.")
    else:
        print("  ❌ 점검 필요:")
        if dead:
            print("     - 신호 없는 센서: "
                  + ", ".join(LABEL[v.expected] for v in dead)
                  + "  → 배선/전원 확인 (docs/hardware_troubleshooting.md).")
        if swaps:
            print("     - 방향 어긋남(좌우 바뀜 의심): "
                  + ", ".join(f"{v.direction.upper()}→{v.nearest}" for v in swaps)
                  + "  → 해당 센서 TRIG/ECHO 핀 또는 장착 위치 서로 확인.")
        if retry:
            print("     - 재시도 권장: "
                  + ", ".join(v.direction.upper() for v in retry)
                  + "  → 장애물 위치/거리 조정 후 다시 실행.")
    print("=" * 64)


def run_interactive() -> int:
    require_gpio()
    print_guide()
    us = Ultrasonics()
    us.setup()
    results: list[DirectionVerdict] = []
    try:
        with tracer.run("ultrasonic-direction-check"):
            print("[0] 기준선: 세 방향 모두 비운 상태에서 측정합니다. Enter...")
            input()
            baseline = poll_stable(us)
            print(f"    baseline: {_fmt(baseline)}")
            dead = [k for k, val in baseline.items() if val is None]
            if dead:
                print(f"    ⚠️  신호 없는 센서: {', '.join(LABEL[k] for k in dead)} "
                      f"— 그래도 방향 점검은 계속합니다.")
            print()

            for direction in DIRECTION_ORDER:
                print_step_guide(direction)
                input()
                readings = poll_stable(us)
                verdict = classify_direction(direction, readings)
                results.append(verdict)
                print_verdict(verdict)
                print()

            print_summary(results)
    finally:
        us.cleanup()
        if GPIO is not None:
            GPIO.cleanup()
        print("GPIO cleanup 완료.")
    return 0 if results and all(v.status == PASS for v in results) else 1


# --------------------------------------------------------------------- #
# Demo flow (no hardware) — lets the SW lead preview the tool + verdicts
# before handing it to the hardware team.
# --------------------------------------------------------------------- #
def run_demo() -> int:
    print_guide()
    print("(--demo: 하드웨어 없이 예시 측정값으로 판정 흐름만 보여줍니다)\n")

    scenarios = {
        "정상 (모두 올바름)": {
            "front": {"front": 12.0, "left45": 90.0, "right45": 95.0},
            "left": {"front": 80.0, "left45": 13.0, "right45": 92.0},
            "right": {"front": 85.0, "left45": 88.0, "right45": 11.0},
        },
        "좌우 배선 바뀜": {
            "front": {"front": 12.0, "left45": 90.0, "right45": 95.0},
            "left": {"front": 80.0, "left45": 91.0, "right45": 13.0},
            "right": {"front": 85.0, "left45": 12.0, "right45": 89.0},
        },
        "오른쪽 센서 죽음": {
            "front": {"front": 12.0, "left45": 90.0, "right45": None},
            "left": {"front": 80.0, "left45": 13.0, "right45": None},
            "right": {"front": 85.0, "left45": 88.0, "right45": None},
        },
    }

    for title, steps in scenarios.items():
        print("-" * 64)
        print(f" 시나리오: {title}  (예시 측정값으로 자동 진행)")
        print("-" * 64)
        results: list[DirectionVerdict] = []
        for direction in DIRECTION_ORDER:
            # Same step-by-step guide the operator sees on a real run.
            print_step_guide(direction)
            verdict = classify_direction(direction, steps[direction])
            results.append(verdict)
            print_verdict(verdict)
            print()
        print_summary(results)
        print()
    # --demo is an illustrative preview (it intentionally shows failing
    # scenarios), so it always exits 0 — it is not a pass/fail gate.
    return 0


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="3-sensor ultrasonic integration + direction/placement check."
    )
    p.add_argument(
        "--demo",
        action="store_true",
        help="run scripted scenarios off the Pi (no GPIO) to preview the flow",
    )
    return p.parse_args()


def main() -> None:
    args = parse_args()
    sys.exit(run_demo() if args.demo else run_interactive())


if __name__ == "__main__":
    main()
