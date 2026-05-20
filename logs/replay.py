"""Replay a JSONL trace through the state machine.

Two use cases:
  1. ``python logs/replay.py <path>``           - run the recorded sensor
       stream through a fresh ``WallFollowerSM`` and report transitions and
       final state. Lets us verify state-machine behavior off the Pi --
       critical while LEFT45 / RIGHT45 are still being wired.

  2. ``python logs/replay.py <path> --latency`` - pair each RED-camera
       event with the next zero-PWM motor event and print reaction-delay
       statistics. This is how the grading-criterion #4 metric is
       computed; **never** measure it with a stopwatch.

The format is the one written by ``logs/trace.py`` (one JSON object per
line, ``{t, type, data}``).
"""

from __future__ import annotations

import argparse
import json
import statistics
import sys
from pathlib import Path

# Allow `python logs/replay.py ...` from anywhere by making sure the repo
# root is on sys.path before importing project modules.
_REPO_ROOT = Path(__file__).resolve().parent.parent
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))

# Local import — keeps this script usable even off the Pi.
from algorithm.wall_follower_sm import WallFollowerSM  # noqa: E402


def load(path: str | Path) -> list[dict]:
    p = Path(path)
    if not p.exists():
        print(f"[ERR] no such file: {p}")
        sys.exit(1)
    out: list[dict] = []
    for line in p.read_text(encoding="utf-8").splitlines():
        line = line.strip()
        if not line:
            continue
        try:
            out.append(json.loads(line))
        except json.JSONDecodeError:
            continue
    return out


def latency_report(records: list[dict]) -> None:
    """Pair each STOP camera event with the next zero-PWM motor event."""
    deltas_ms: list[float] = []
    pending_t: float | None = None
    for r in records:
        t = float(r.get("t", 0.0))
        if r.get("type") == "camera":
            sig = r["data"].get("signal")
            if sig == "STOP" and pending_t is None:
                pending_t = t
            elif sig != "STOP":
                pending_t = None
        elif r.get("type") == "motor" and pending_t is not None:
            d = r["data"]
            if abs(float(d.get("left_pwm", 0))) < 0.01 and abs(float(d.get("right_pwm", 0))) < 0.01:
                deltas_ms.append((t - pending_t) * 1000.0)
                pending_t = None

    if not deltas_ms:
        print("[latency] no RED -> STOP-motor pairs found")
        return
    print(f"[latency] RED -> motor-stop pairs: {len(deltas_ms)}")
    print(f"          mean = {statistics.mean(deltas_ms):7.1f} ms")
    print(f"          stdev= {statistics.pstdev(deltas_ms):7.1f} ms")
    print(f"          min  = {min(deltas_ms):7.1f} ms")
    print(f"          max  = {max(deltas_ms):7.1f} ms")
    if max(deltas_ms) > 200:
        print("[latency] WARN -- some reactions >200ms; tune loop frequency or perception path")


def replay(records: list[dict], verbose: bool = False) -> None:
    """Feed sensor + camera events back into a fresh state machine."""
    sm = WallFollowerSM()
    latest: dict[str, float | None] = {"front": None, "left45": None, "right45": None}
    last_signal = "UNKNOWN"
    transitions = 0
    last_state = sm.state

    for r in records:
        typ = r.get("type")
        d = r.get("data", {})

        if typ == "ultrasonic":
            sensor = d.get("sensor")
            if sensor in latest:
                latest[sensor] = d.get("filtered_cm") if d.get("valid") else None
        elif typ == "camera":
            last_signal = d.get("signal", "UNKNOWN")
        else:
            continue

        cmd = sm.step(latest["front"], latest["left45"], latest["right45"], last_signal)
        if verbose:
            t = float(r.get("t", 0.0))
            print(
                f"  t={t:7.3f}  {typ:10s} sig={last_signal:7s}  "
                f"-> {sm.state:14s} {cmd.action}"
            )
        if sm.state != last_state:
            transitions += 1
            last_state = sm.state

    print(f"[replay] state transitions: {transitions}   final state: {sm.state}")


def summary(records: list[dict]) -> None:
    counts: dict[str, int] = {}
    for r in records:
        counts[r.get("type", "?")] = counts.get(r.get("type", "?"), 0) + 1
    print(f"[load] {len(records)} records")
    for typ, n in sorted(counts.items()):
        print(f"       {typ:14s} {n}")


def main() -> int:
    ap = argparse.ArgumentParser(description="Replay JSONL trace through state machine")
    ap.add_argument("path", help="path to a logs/runs/*.jsonl file")
    ap.add_argument("--latency", action="store_true", help="compute RED -> motor-stop latencies")
    ap.add_argument("--verbose", action="store_true", help="print every replay step")
    ap.add_argument("--no-replay", action="store_true", help="skip the replay (--latency only)")
    args = ap.parse_args()

    records = load(args.path)
    summary(records)
    if args.latency:
        latency_report(records)
    if not args.no_replay:
        replay(records, verbose=args.verbose)
    return 0


if __name__ == "__main__":
    sys.exit(main())
