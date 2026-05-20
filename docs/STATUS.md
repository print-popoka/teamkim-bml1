# Project status (snapshot)

_Updated 2026-05-17 — after PI smooth-drive feedback + first hardware_check_

## Where we are

Architecture skeleton is **in place and importable**. Calibration tools
are ready. All numeric constants in the runtime code are placeholders
that get filled in from the sample-maze test next week. Two of three
ultrasonic sensors need a wiring fix from the hardware team before the
test.

## Repo layout

```
.
├── hardware_check.py         # Pi-side: verify all wiring before any session
├── main.py                   # entry: full run loop (--dry-run works off Pi)
│
├── camera/                   # LEGACY scripts + calibration tools (kept)
│   ├── yolo.py               #   YOLOv8n demo (lecture)
│   ├── hsv.py                #   basic HSV (lecture)
│   ├── hsv_circle.py         #   tuned HSV + circularity (primary detector)
│   ├── yolo_hsv.py           #   YOLO+HSV hybrid (fallback for noisy maze)
│   └── hsv_picker.py         #   click-to-read HSV (on-site re-calibration)
│
├── motor/                    # LEGACY + tools
│   ├── motor.py              #   forward-test (lecture)
│   └── motor_calibration.py  #   interactive 4-test calibration
│
├── sensor/                   # LEGACY + tools
│   ├── ultrasonic.py         #   single-sensor demo (lecture)
│   └── ultrasonic_noise.py   #   noise characterization (200 samples)
│
├── hal/                      # NEW — hardware abstraction
│   ├── ultrasonics.py        #   3-sensor manager, median filter, warmup discard
│   └── motors.py             #   L298N with smooth drive(L,R) primitive
│
├── perception/               # NEW — sensor -> meaning
│   └── traffic_light.py      #   TrafficLightDetector class (uses hsv_circle logic)
│
├── control/                  # NEW — smooth motion
│   └── wall_follow.py        #   PD controller with corner anticipation,
│                             #   clearance guard, pivot fallback
│
├── algorithm/                # NEW — high-level
│   └── wall_follower_sm.py   #   state machine: INIT / FOLLOWING / STOPPED_AT_RED
│                             #                  / PIVOTING
│
├── logs/                     # NEW — structured logging
│   ├── trace.py              #   JSONL tracer singleton
│   └── runs/                 #   per-run log files (gitignored)
│
├── docs/
│   ├── test_day_checklist.md          #   take this to the sample maze
│   ├── hardware_troubleshooting.md    #   multimeter recipe for LEFT45/RIGHT45
│   └── STATUS.md                      #   this file
│
└── tests/                    # NEW — pytest behavioral tests
    ├── test_wall_follow.py   #   centering / clearance / corner / pivot (10)
    └── test_state_machine.py #   INIT / FOLLOWING / RED / GREEN / pivot (9)
```

(plus ``logs/replay.py`` — JSONL → state machine offline replay + latency report)

## What works without hardware (right now, on any machine with Python)

```bash
python logs/trace.py                              # demo trace write
python logs/trace.py show logs/runs/<file>.jsonl  # pretty-print
python main.py --dry-run --duration 5             # smoke test the loop
python logs/replay.py logs/runs/<file>.jsonl --latency  # replay + latency
pytest tests/ -q                                  # 19 behavioral tests
```

`main.py --dry-run` exercises the entire perception/control/algorithm
chain with mocked sensors (always returns None, signal=UNKNOWN). State
machine should sit in INITIALIZING the whole time, log a session-end
event, and exit cleanly.

## What needs hardware

- `hardware_check.py` — wiring verification
- `sensor/ultrasonic_noise.py` — distance-vs-noise data
- `motor/motor_calibration.py` — PWM / speed / drift / turn calibration
- `camera/hsv_picker.py` — HSV under sample-maze lighting
- `main.py` (full mode) — the real ride

## Calibration status

| Constant | Where | Status |
|---|---|---|
| HSV thresholds (RED/GREEN) | `perception/traffic_light.py` | ✅ tuned 2026-05-17 from 18 printed-paper samples |
| Ultrasonic median window | `hal/ultrasonics.py` | ⏳ placeholder, refine after test day |
| Min PWM L/R | `hal/motors.py` | ⏳ placeholder (20/20) |
| Speed table | `hal/motors.py` | ⏳ placeholder (25 cm/s at 50%) |
| Drift trim | `hal/motors.py` | ⏳ placeholder (1.0/1.0) |
| Turn rate | `hal/motors.py` | ⏳ placeholder (150 deg/s) |
| Wall-follow PD gains | `control/wall_follow.py` | ⏳ placeholder, retune on real maze |
| Corner-anticipate distance | `control/wall_follow.py` | ⏳ placeholder (25 cm) |
| SAFE_MARGIN_CM | `control/wall_follow.py` | ⏳ placeholder (4 cm) |

## Hardware status (per the `hardware_check.py` run that motivated this turn)

| Channel | Pins | Result | Action |
|---|---|---|---|
| Ultrasonic FRONT | 23/24 | ✅ works (1st sample was warmup outlier; now discarded) | none |
| Ultrasonic LEFT45 | 25/8 | ❌ echo never went HIGH | hardware team — see `docs/hardware_troubleshooting.md` §LEFT45 |
| Ultrasonic RIGHT45 | 7/12 | ❌ echo idle HIGH | hardware team — see `docs/hardware_troubleshooting.md` §RIGHT45 |
| Motors L/R | 17/27/18, 22/5/19 | ✅ GPIO/PWM commands OK | visual confirm direction (next Pi session) |

## Open inquiries (waiting on user action)

| # | Item | What's needed |
|---|---|---|
| 6 | Hardware team wiring confirmation | Confirm 3 ultrasonics + L298N wiring after fixes |
| 7 | MPU6050 IMU decision | Buy or skip — only matters if turn rate proves unreliable |
| 11 | Hardware team: LEFT45/RIGHT45 wiring | Specific to the FAIL items in the last `hardware_check.py` |

## Locked decisions (from CLAUDE.md, do not relitigate)

- **Algorithm**: right-hand wall-follow (TA confirmed left/right parity ⇒ simply connected maze; Pledge counter NOT needed; structure keeps a slot in case spec changes).
- **Driving style**: smooth-drive only (continuous arcs); in-place pivot is dead-end fallback only.
- **Wall-follow target**: center the car using (right − left); right-hand bias on ties.
- **Camera detector at runtime**: `hsv_circle` logic (via `perception/traffic_light.py`); YOLO is dev-only.
- **Traffic light**: RED→STOP / GREEN→GO only. No yellow.
- **Logging**: every measurement and decision goes through `logs.trace.tracer`. No stopwatching for the camera→motor latency metric — derive it from the JSONL.

## Next actions

1. **Hardware team** fixes LEFT45 + RIGHT45 wiring; rerun `hardware_check.py` until all PASS.
2. **You at the sample maze** (next week): follow `docs/test_day_checklist.md`. Send the numbers back.
3. **Me**: bake measured constants into `hal/motors.py`, `hal/ultrasonics.py`, `control/wall_follow.py`; push.
4. **Together**: first full `python main.py` run; iterate on the trace log.
