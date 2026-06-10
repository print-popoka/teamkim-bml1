# Project status (snapshot)

_Updated 2026-06-11 — **completion-first control redesign** after 2026-06-10
field runs (failures: U-turn wall hits #1, zigzag→side hits #2, missed
corners, false EXITED stops). Changes: (1) closed-loop **wall-reacquire
turn** replaces the fixed junction commit — straight past the wall end,
then arc right until the wall is seen again; (2) left openings ignored
(right-hand purity); (3) zigzag damping — KP↓ KD↑ deadband↑, centering
output capped at ±0.5, per-tick curvature growth limit, accel slew;
(4) None policy — side None holds last valid until dead-mirroring (no
phantom-400 swerves), front None holds then caps speed; (5) stuck/pivot
judgments None-robust + two-phase recovery (reverse → turn, escalating);
(6) **EXITED self-stop removed entirely** (user decision — run ends by
power cut / --duration). 106 pytest pass; sim sweep 179/180 EXIT_REACHED
across 2 mazes x {0,15,30}% sensor-dropout x 30 seeds (only miss: 1 seed
at the extreme 30% rate). Prior 2026-06-04: SM-1/CTRL-1/PERC-1 pass._

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
├── ultrasonic_direction_check.py  # Pi-side: 3-sensor integration + L/R/front
│                             #   DIRECTION check (catches a left<->right swap);
│                             #   --demo previews it off-Pi
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
│                             #   / PIVOTING / RECOVERING (no EXITED — the
│                             #   self-stop was removed 2026-06-10)
│
├── logs/                     # NEW — structured logging
│   ├── trace.py              #   JSONL tracer singleton
│   └── runs/                 #   per-run log files (gitignored)
│
├── docs/
│   ├── test_day_checklist.md          #   take this to the sample maze
│   ├── hardware_troubleshooting.md    #   multimeter recipe for LEFT45/RIGHT45
│   ├── ultrasonic_direction_check.md  #   hardware-team guide for the L/R/front check
│   └── STATUS.md                      #   this file
│
└── tests/                    # NEW — pytest behavioral tests
    ├── test_wall_follow.py   #   centering / clearance / corner / pivot
    │                         #   + speed profile / CRUISE knob / coupling (29)
    └── test_state_machine.py #   INIT / FOLLOWING / RED / GREEN / pivot
                              #   + full-chain speed/gating integration (11)
```

(plus ``logs/replay.py`` — JSONL → state machine offline replay + latency report)

## What works without hardware (right now, on any machine with Python)

```bash
python logs/trace.py                              # demo trace write
python logs/trace.py show logs/runs/<file>.jsonl  # pretty-print
python main.py --dry-run --duration 5             # smoke test the loop
python logs/replay.py logs/runs/<file>.jsonl --latency  # replay + latency
pytest tests/ -q                                  # 106 behavioral tests
python tools/maze_sim.py --maze uturn --none-rate 0.25  # U-turn case + dropout noise
python ultrasonic_direction_check.py --demo       # preview the 3-sensor direction check off-Pi
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
| **CRUISE speed knob** | `control/wall_follow.py` | ⏳ single hardware-day dial (34.0, completion-first); whole speed table derives from it |
| Corner-anticipate distance | `control/wall_follow.py` | ⏳ 45 cm at reference cruise; SPEED-AWARE (grows with CRUISE via ANTICIPATE_GAIN) |
| Wall-reacquire turn (REACQ_*) | `control/wall_follow.py` | ⏳ straight 18 ticks / curvature −0.5 / timeout 110 — sim-tuned, Pi-validate |
| SAFE_MARGIN_CM | `control/wall_follow.py` | ⏳ placeholder (4 cm) |

## Hardware status (per the `hardware_check.py` run that motivated this turn)

> The ultrasonic numbers below are from a **sanity check, not a positional
> calibration**. The sensors were sitting on a bench at an arbitrary
> distance; the takeaway is "FRONT alive, LEFT45/RIGHT45 dead", not any
> particular cm reading. Real distance-vs-truth data will come from the
> sample-maze test next week using `sensor/ultrasonic_noise.py --true N`.



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
- **Completion over speed** (user, 2026-06-10): 95/100 finishes beats 10 fast finishes. CRUISE and all maneuver speeds are tuned conservatively.
- **No self-stop / exit auto-detection** (user, 2026-06-10): a false EXITED kills the run; a true exit is handled by cutting power. STOPPED_AT_RED stays (graded requirement).
- **Right-hand purity**: LEFT openings are never committed into; left turns only come from front-wall corner anticipation.
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
