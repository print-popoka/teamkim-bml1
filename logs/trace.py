"""Structured logging (JSONL) for the maze robot.

Writes one JSON object per line to a per-run log file under ``logs/runs/``.
Designed for:

  * **Offline replay & debugging** — load the JSONL, walk the timeline,
    re-run the algorithm without the robot.
  * **Reaction-delay measurement** — pair a ``camera`` event with the next
    ``motor`` event, take the time delta. Grading criterion #4.
  * **Post-run analysis** — when a run fails, the log answers "what was
    the robot seeing right before it crashed?".

Why not stdlib ``logging``?
    ``logging`` targets human-readable text. We need machine-parseable
    structured data with strict schemas across event types. A thin JSONL
    writer + per-type helpers is simpler than ``logging.Handler`` +
    ``Formatter`` gymnastics, and faster (no string formatting).

Wire format
-----------
One JSON object per line, schema::

    {"t": <float seconds since session start>,
     "type": "ultrasonic" | "camera" | "decision" | "motor"
           | "state" | "info" | "session_start" | "session_end",
     "data": { ... type-specific payload ... }}

Per-type ``data`` shapes (see helper methods for authoritative signatures)::

    ultrasonic    {sensor, raw_cm, filtered_cm, valid}
    camera        {signal, red_area, green_area, frame_id}
    decision      {state, action, reason}
    motor         {left_pwm, right_pwm, direction}
    state         {state, from, reason}
    info          {msg, ...arbitrary}
    session_start {name, started_wall_unix, started_wall_iso}
    session_end   {duration_s}

Usage
-----
::

    from logs.trace import tracer

    tracer.start("right-hand-test-01")
    tracer.ultrasonic(sensor="front", raw_cm=12.4, filtered_cm=12.6, valid=True)
    tracer.camera(signal="STOP", red_area=520, green_area=8)
    tracer.motor(left_pwm=0, right_pwm=0, direction="stop")
    tracer.stop()

    # or as a context manager (auto-stop on exception):
    with tracer.run("right-hand-test-01"):
        ...

CLI
---
::

    python logs/trace.py                # write a small demo log
    python logs/trace.py show <file>    # pretty-print an existing log

Threading
---------
This module is **not thread-safe by default**. The main robot loop is
single-threaded; if we later add a sensor-polling thread we will add a
``threading.Lock`` around ``_write``. Until then we save the overhead.
"""

from __future__ import annotations

import json
import time
from dataclasses import dataclass
from pathlib import Path
from typing import IO, Any

# Resolve to <repo>/logs/runs/ regardless of CWD.
LOG_DIR: Path = Path(__file__).parent / "runs"


@dataclass
class _Session:
    name: str
    started_monotonic: float
    started_wall: float
    file_path: Path
    fh: IO[str]


class Tracer:
    """Module-level tracer singleton, importable from anywhere.

    Calls made when no session is active are silent no-ops, so production
    code can call ``tracer.foo(...)`` unconditionally and tests / dry-runs
    that skip ``tracer.start()`` simply don't write anything.
    """

    def __init__(self) -> None:
        self._session: _Session | None = None

    # ------------------------------------------------------------------ #
    # Session lifecycle
    # ------------------------------------------------------------------ #
    def start(self, name: str = "run") -> Path:
        """Open a new log file. Returns the path so callers can announce it."""
        if self._session is not None:
            raise RuntimeError(
                f"Tracer already active (session={self._session.name!r}). "
                "Call stop() first or use the run() context manager."
            )
        LOG_DIR.mkdir(parents=True, exist_ok=True)
        ts = time.strftime("%Y%m%d_%H%M%S", time.localtime())
        safe = "".join(c if c.isalnum() or c in "-_" else "_" for c in name)
        path = LOG_DIR / f"{ts}_{safe}.jsonl"
        # buffering=1 -> line-buffered (text mode); flushes every newline.
        fh = path.open("a", buffering=1, encoding="utf-8")
        now_mono = time.monotonic()
        now_wall = time.time()
        self._session = _Session(
            name=name,
            started_monotonic=now_mono,
            started_wall=now_wall,
            file_path=path,
            fh=fh,
        )
        self._write(
            "session_start",
            {
                "name": name,
                "started_wall_unix": now_wall,
                "started_wall_iso": time.strftime(
                    "%Y-%m-%dT%H:%M:%S", time.localtime(now_wall)
                ),
            },
        )
        return path

    def stop(self) -> None:
        if self._session is None:
            return
        self._write("session_end", {"duration_s": round(self._elapsed(), 6)})
        self._session.fh.close()
        self._session = None

    def is_active(self) -> bool:
        return self._session is not None

    def run(self, name: str = "run") -> "_RunCtx":
        """Context manager: ``with tracer.run('name'): ...``."""
        return _RunCtx(self, name)

    # ------------------------------------------------------------------ #
    # Internals
    # ------------------------------------------------------------------ #
    def _elapsed(self) -> float:
        if self._session is None:
            return 0.0
        return time.monotonic() - self._session.started_monotonic

    def _write(self, type_: str, data: dict[str, Any]) -> None:
        if self._session is None:
            return  # no-op when inactive
        record = {"t": round(self._elapsed(), 6), "type": type_, "data": data}
        self._session.fh.write(json.dumps(record, ensure_ascii=False) + "\n")

    # ------------------------------------------------------------------ #
    # Typed event helpers — one per event type
    # ------------------------------------------------------------------ #
    def ultrasonic(
        self,
        sensor: str,
        raw_cm: float | None,
        filtered_cm: float | None = None,
        valid: bool = True,
    ) -> None:
        self._write(
            "ultrasonic",
            {
                "sensor": sensor,
                "raw_cm": raw_cm,
                "filtered_cm": filtered_cm,
                "valid": valid,
            },
        )

    def camera(
        self,
        signal: str,
        red_area: int = 0,
        green_area: int = 0,
        frame_id: int | None = None,
    ) -> None:
        self._write(
            "camera",
            {
                "signal": signal,
                "red_area": red_area,
                "green_area": green_area,
                "frame_id": frame_id,
            },
        )

    def decision(self, state: str, action: str, reason: str = "") -> None:
        self._write(
            "decision",
            {"state": state, "action": action, "reason": reason},
        )

    def motor(
        self,
        left_pwm: float,
        right_pwm: float,
        direction: str,
    ) -> None:
        self._write(
            "motor",
            {
                "left_pwm": left_pwm,
                "right_pwm": right_pwm,
                "direction": direction,
            },
        )

    def state(
        self,
        state: str,
        from_state: str | None = None,
        reason: str = "",
    ) -> None:
        self._write(
            "state",
            {"state": state, "from": from_state, "reason": reason},
        )

    def info(self, msg: str, **fields: Any) -> None:
        self._write("info", {"msg": msg, **fields})


@dataclass
class _RunCtx:
    tracer: Tracer
    name: str

    def __enter__(self) -> Path:
        return self.tracer.start(self.name)

    def __exit__(self, *exc: Any) -> None:
        self.tracer.stop()


# Module-level singleton.
tracer = Tracer()


# ====================================================================== #
# CLI helpers — demo + show
# ====================================================================== #
def _demo() -> None:
    """Write a short demo log so the user can verify the pipeline works."""
    import random

    with tracer.run("demo") as path:
        for i in range(20):
            tracer.ultrasonic(
                sensor=random.choice(["front", "left", "right"]),
                raw_cm=round(random.uniform(5, 100), 2),
                filtered_cm=round(random.uniform(5, 100), 2),
            )
            if i % 5 == 0:
                tracer.camera(
                    signal=random.choice(["STOP", "GO", "UNKNOWN"]),
                    red_area=random.randint(0, 500),
                    green_area=random.randint(0, 500),
                    frame_id=i,
                )
            time.sleep(0.05)
        tracer.info("demo done", n_events=20)
    print(f"[OK] wrote {path}")
    print(f"     inspect with: python logs/trace.py show {path}")


def _show(path_str: str) -> None:
    """Pretty-print an existing JSONL log."""
    p = Path(path_str)
    if not p.exists():
        print(f"[ERR] no such file: {p}")
        return
    with p.open(encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            r = json.loads(line)
            t = r.get("t", 0.0)
            typ = r.get("type", "?")
            data = r.get("data", {})
            print(f"  t={t:8.3f}  {typ:14s}  {data}")


def _cli() -> None:
    import sys

    args = sys.argv[1:]
    if not args:
        _demo()
        return
    if args[0] == "show" and len(args) >= 2:
        _show(args[1])
        return
    print(
        "Usage:\n"
        "  python logs/trace.py              # write a demo log\n"
        "  python logs/trace.py show <path>  # pretty-print a log"
    )


if __name__ == "__main__":
    _cli()
