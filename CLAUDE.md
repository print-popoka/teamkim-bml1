# Copyright © Hyunjung Kim. All rights reserved.

## teamkim-bml1

Basic Mobile Lab 1 — Teamkim 팀.

## Hardware

- **Raspberry Pi 4B rev 1.5** — BCM2711, 1.5GHz quad-core Cortex-A72 (aarch64),
  4GB RAM, MicroSD. CPU only, no GPU → YOLOv8n is the largest usable model.
- **Camera**: OV5647 (Pi Camera Rev 1.3), 5MP, horizontal FOV ~54° (narrow —
  traffic light leaves frame easily), fixed focus f/2.9, CSI ×1. Using 640×480.
- **L298N motor driver** — 2 channels (A/B), 2A continuous per channel,
  12V motor supply.
- **DC motors ×4** (4-wheel), paired left/right. **No encoders** (core
  algorithm constraint — no odometry, time-based estimation only).
- **HC-SR04 ultrasonic ×3** — front-center, front-left ~45°, front-right ~45°.
  Range 2–400cm, 40kHz, 5V (Echo also 5V, no divider used). Round-robin
  ≈10Hz, ~10ms cooldown between pings, 30ms timeout.

## Runtime environment

- OS: Raspberry Pi OS (Debian-based), Python **3.13**
- User `team2`, host `MobileLab`, Desktop `/home/team2/Desktop/`
  → this is the confirmed `output_dir` for `camera/yolo.py`
- pip is `externally-managed` → **always** `--break-system-packages`
- `/tmp` is tmpfs 1.9G (small) → set `TMPDIR=/home/team2` for big installs

## GPIO (BCM) — RECOMMENDED pin map (relay to hardware team)

Not as-built. This is the spec the user should hand to the hardware team.
Ultrasonic uses **independent TRIG/ECHO per sensor** (6 pins) — chosen for
robustness and clean per-sensor code. None collide with motor pins.

| Function | Pin |
|---|---|
| Motor A IN1 / IN2 | GPIO 17, 27 |
| Motor B IN3 / IN4 | GPIO 22, 5 |
| ENA / ENB (PWM) | GPIO 18, 19 |
| Ultrasonic FRONT  TRIG / ECHO | GPIO 23, 24 |
| Ultrasonic LEFT45 TRIG / ECHO | GPIO 25, 8 |
| Ultrasonic RIGHT45 TRIG / ECHO | GPIO 7, 12 |

> Legacy `sensor/ultrasonic.py` uses the single FRONT pair (23/24) only —
> kept as baseline. New `hal/ultrasonics.py` will drive all three.

## Constraints

- CPU only — no GPU. YOLOv8n is the largest model usable.
- Pip installs **must** use `--break-system-packages`.
- Set `TMPDIR=/home/team2` before installing (avoids /tmp space issues).
- **Do not run YOLO and HSV simultaneously** — single-CPU contention will stall both.
- **L298N 2A/channel limit**: pairing 2 motors per channel may exceed 2A on
  stall. Confirm with current measurement at first integration test; 2× L298N
  is the safe fallback. Wiring not finalized.

## Install (on the Pi)

```bash
sudo apt update -y
sudo apt install -y vim python3-opencv python3-picamera2 libcamera-apps python3-pip

export TMPDIR=/home/team2
pip3 install numpy --break-system-packages
pip3 install torch torchvision --extra-index-url https://download.pytorch.org/whl/cpu --break-system-packages
pip install ultralytics --no-deps --break-system-packages
```

## Headless autostart (battery-only at the sample maze)

The Pi at the sample maze has no keyboard/mouse. A systemd unit
(`scripts/teamkim-bml1.service`) runs `main.py` on every boot after a
short delay. Install once with `bash scripts/install_autostart.sh`,
then plug-battery == one drive run, trace lands on the SD card.

Full workflow + tunables + recovery: `docs/headless_boot.md`.

This **does not** cover interactive calibration tools
(`motor_calibration.py`, per-distance `ultrasonic_noise.py` series) —
those need keyboard or phone-SSH.

## Run

```bash
rpicam-hello -t 3000              # camera sanity check

python camera/yolo.py             # YOLOv8n object detection
python camera/hsv.py              # HSV traffic-light (basic)
python camera/hsv_circle.py       # HSV + circle filter

python motor/motor.py             # forward test
python sensor/ultrasonic.py       # distance loop
```

## NEXT SESSION — start here

**Top priority: sensor noise calibration — BOTH ultrasonic AND camera.**
Before any algorithm code, get clean trustworthy signals from both:
- 3× HC-SR04: stable distances (median/moving-average, outlier rejection)
- Camera: stable traffic-light decision (HSV under real lighting, temporal
  smoothing, exposure/white-balance noise, ROI to cut false positives)

Agreed algorithm direction (settled, do not re-litigate):
- **Right-hand wall-following is the MAIN algorithm.** Pledge's angle counter
  is unreliable without a heading sensor and gives no benefit on a
  simply-connected maze.
- Build the architecture so the Pledge counter is a **pluggable layer**
  (off by default; switch on only if maze has islands or an IMU is added).
- Competitive edge = execution quality (PD wall-follow, closed-loop turns,
  sensor filtering, replay debugging, low camera→motor latency), not exotic
  algorithm choice.

Open items to pursue in parallel (user issues these as orders/inquiries):
1. Ask TA/instructor whether the evaluation maze is simply connected or has
   loops/islands — this finalizes the algorithm.
2. Decide whether to add an MPU6050 IMU (~₩2000) for reliable heading;
   unlocks Pledge/Trémaux. User's call; Claude only presents trade-offs.

First concrete tasks next session (parallel):
1. Ultrasonic noise characterization + filtering (median/moving-average,
   outlier rejection), with a measurement guide for the user.
2. Camera noise calibration: measure real HSV of lit red/green (and yellow if
   enabled) via hsv_picker.py under test lighting, set ranges from data,
   add temporal smoothing + ROI; quantify false-positive/flicker rate.

## Role boundary (IMPORTANT)

The user is the **software design lead**, not hardware. They do NOT physically
wire or assemble the robot and may not know the exact as-built wiring.

- Pin maps / wiring / sensor placement in this doc are **recommended specs to
  hand to the hardware team**, NOT confirmed as-built facts.
- When hardware detail matters, state it as "recommend the hardware team do X"
  so the user can relay it as an order, then reconcile once they report back
  what was actually built.
- Never assume a pin assignment is real until the user confirms the hardware
  team implemented it.

The user owns **all software design + implementation** and **issues hardware
orders** (defines the spec, the hardware team builds it). This framing is also
used in the user's personal portfolio — keep commits, docs, and architecture
consistent with "SW fully owned by user; HW spec'd then ordered out".

## Project Priority

**The single goal is: complete the maze (미로 탈출).** Everything — code structure,
sensor tuning, motor calibration — serves that goal. Lecture-slide fidelity is
secondary once the baseline scripts exist.

## Locked decisions (do not redo without asking)

- **Algorithm** (CONFIRMED via TA, 2026-05-17): **right-hand wall-following
  alone**. TA stated left-hand vs right-hand will yield identical times by
  design ("운에 의존하지 않을 것") — this implies the maze is simply
  connected with no islands, so plain wall-following is provably complete.
  Pledge counter is NOT needed. Architecture still keeps a counter slot as a
  pluggable layer (default OFF) in case the spec changes, but no engineering
  effort goes into it.
- **Corner geometry** (CONFIRMED via TA): 90° corners only. No oblique angles.
  Turn state machine simplifies — closed-loop turns can be driven by
  "side wall appears" or "front wall disappears" events, no angle tracking.
- **Maze wall material** (CONFIRMED via TA): 우드락 (foam board). Smooth
  surface → ultrasonic should reflect well, but verify 45° angled returns
  in the ultrasonic calibration phase.
- **Traffic light source** (CONFIRMED via TA): printed paper (same as alpha
  test). Current HSV calibration on printed paper remains valid; verify with
  hsv_picker.py on the actual evaluation print on test day.
- **Maze corridor width** (CONFIRMED via TA, 2026-05-17): baseline 25–30 cm
  with sections that narrow then widen. Wall height 10cm+ (above chassis,
  safe for any ultrasonic mount height). **Chassis is 15cm wide, 18cm with
  wheels → only 3.5cm clearance per side at narrowest. Tight.** Implications:
    * Wall-following must hold the car within ±2 cm of center, not just
      "follow the right wall". Switch direct PD target from "right-wall
      distance" to **(right - left) error → 0** (center the car), with
      a clearance guard that overrides if either side is < safe_margin_cm.
    * Forward speed must be reduced in narrowing sections (detect when
      both side distances drop together → slow PWM).
    * Ultrasonic accuracy ±1–2 cm is mandatory, not nice-to-have.
- **Traffic light operation** (CONFIRMED via TA): manually operated by judge,
  count TBD. → no timing/cycle assumption; react frame-by-frame to whatever
  is currently shown. Multiple lights handled by running the same detector
  per frame regardless of count.
- **Start orientation** (CONFIRMED via TA): free choice. Convention: place
  the car facing into the maze with a wall on the right side (matches
  right-hand wall-following). Hardware order to team: "start position with
  right wall within ~10cm". Initialization logic should also handle "no
  right wall detected → drive forward until one appears".
- **Driving style — smooth, never stop-and-turn** (PI feedback, 2026-05-17):
  the car must navigate corners as continuous arcs, not as "stop → in-place
  rotate → resume". This means:
    * The motor primitive is `drive(left_pwm, right_pwm)` — both wheels are
      *always* turning while moving; direction is encoded as the differential,
      not as a discrete state.
    * Higher-level helpers `forward(speed)`, `arc_left/right(speed, curvature)`
      all wrap that primitive. No `stop_then_turn()` in the normal path.
    * Cornering = anticipate from front distance; as front distance drops,
      slow the inside wheel and curve through the corner. Wall-following PD
      blends seamlessly into corner arcs (corner is just a tight setpoint).
    * In-place pivot stays in the codebase as a **fallback only** for
      dead-end U-turns (tight 180° in a 25cm corridor where an arc can't
      fit). Never used on normal 90° corners.
  Rationale: smoother driving directly improves the three grading axes
  (completion time, collision count, stability) and the camera→motor
  reaction-delay metric (no discrete state transition latency).

## Professor's common-pitfall tips (2026-05-17) and how we address each

| Pitfall the PI observed in other teams | Our defense |
|---|---|
| Can't smoothly hold the center of the corridor | PD on `right - left` in `control/wall_follow.py` |
| Constant micro-correcting → falls behind on time | **`DEADBAND_CM`** — small errors produce **zero** centering correction. Plus **`MAX_DERROR_CM`** caps the D-term so a sensor jump can't spike steering for one bad tick. Plus error-magnitude **speed scaling** down to `SPEED_SCALE_FLOOR` so big corrections happen at a sustainable speed. |
| Fails to recognize 90° turns / intersections | **Junction commit**: on the rising edge of `r > JUNCTION_CM` (or `l`), the controller locks in `JUNCTION_COMMIT_CURVATURE` for `JUNCTION_COMMIT_TICKS` ticks at `JUNCTION_COMMIT_SPEED`. Stops the car from "smoothing through" a tight right opening even if mid-rotation sensor readings get weird. Tests: `test_junction_*`. |
| Stops then turns at 90° corners (massive time loss) | Locked in CLAUDE.md ("smooth-drive only"). `Motors.drive(L, R)` runs both wheels always; `arc()` slows the inside wheel only; in-place pivot is dead-end fallback only. Plus **zero-clip below MIN_PWM/2** in `Motors._apply_deadband` — when an arc requests a very small inside-wheel PWM, we snap it to 0 instead of promoting to MIN_PWM, so sharp turns stay sharp. |

Whenever editing the controller or state machine, keep these defenses
intact — they're the difference between us and the median team.

### Main-loop performance hygiene

`main.py` runs sensors at full `LOOP_HZ` (10 Hz target) but **throttles
the camera to every CAMERA_EVERY ticks** (~3.3 Hz). Traffic light state
changes are slow (manual operator), so camera doesn't need to keep up.
This frees CPU for the control loop. The last camera signal persists
between captures, so the state machine still sees a recent decision on
non-capture ticks. A periodic `tick_health` trace event records actual
loop duration so we can spot Pi-side overruns in replay.

## Calibration plan — TWO PHASES (do not run all of it at the maze)

Most calibrations don't depend on the maze. Doing them **in advance**
turns sample-maze day from a scramble into a verify-and-tune session.
The robot also arrives already-working, so any hardware surprises are
caught in a safe environment first. See `docs/test_day_checklist.md`
for the full step list.

**Phase A (lab / dorm, before the maze, blocked by hardware wiring fix):**
  - ultrasonic noise per sensor at known distances
  - motor calibration (min PWM / speed / drift / turn rate)
  - L298N stall current with a multimeter
  - Claude bakes measured values into `hal/motors.py` + `hal/ultrasonics.py`
  - first drive in the lab with `python main.py`; tune PD gains
  - end state: a car that drives correctly in a controlled space

**Phase B (sample maze, ~30 min):**
  - verify hardware_check still PASSes
  - re-measure on 우드락 (just one distance per sensor) — capture the
    surface/angle bias
  - HSV re-verify only if venue lighting differs noticeably
  - 2–3 main.py iterations on the actual maze, tuning PD gains and
    recording trace logs
  - record the final run for the presentation video

## Sample-maze test day — must be ready (next week)

This is a one-shot calibration opportunity. By that day we must have ready:

  1. `sensor/ultrasonic_noise.py` (DONE) — run per sensor at fixed distances
     in the actual maze. Capture noise floor, bias, and 45°-mount-angle effect
     on 우드락 walls.
  2. `motor/motor_calibration.py` — minimum PWM, left/right
     asymmetry, straight-drift, 90° turn time.
  3. `camera/hsv_picker.py` (DONE) — re-measure red/green HSV under sample
     maze lighting with sample maze printed traffic light.
  4. **Multimeter** to measure L298N current at motor stall (verify single
     L298N is safe or whether we need 2×).
  5. Notepad / phone for raw measurements; data goes into Claude after.

Plan the order on test day so we don't run out of time. Priority:
  ultrasonic → motor → camera HSV (current is OK, just verify).
- **Ultrasonic layout**: 3 sensors — front-center, front-left (~45°),
  front-right (~45°), each with independent TRIG/ECHO (pin map above).
  Enables wall-following + junction detection.
- **Motor wiring**: 4 DC motors, paired left/right. Specific wiring (single
  L298N parallel vs 2x L298N) deferred until first integration test.
- **Traffic light**: RED→STOP / GREEN→GO only (confirmed — no yellow in real
  test). Yellow handling has been removed from `hsv_circle.py` and
  `yolo_hsv.py`. Do not re-add unless the spec changes.
- **Traffic light safety semantics**: only RED matters. GREEN==UNKNOWN==no-signal
  all collapse to "keep doing what you were doing". HSV thresholds were therefore
  tuned conservative-on-GREEN (occasional false-negative on GREEN is harmless;
  RED must never be missed). Exception handled in main-loop state machine:
  while STOPPED at a red light, only an explicit GREEN releases the brake —
  UNKNOWN holds the stop. To be implemented in algorithm/state machine phase.
- **Main-loop detector = hsv_circle (primary), yolo_hsv (fallback)**. YOLO on
  Pi 4B CPU runs at 1–2 FPS — too slow for a moving car. `hsv_circle.py`
  (HSV + circularity filter) runs at 10–20 FPS and is sufficient for a
  controlled maze with no rogue red/green objects. **Fallback rule**: if a
  maze sample inspection reveals interference (red/green wall patterns, etc.)
  that hsv_circle false-positives on, switch the runtime path to `yolo_hsv.py`
  — no HSV re-calibration needed because both files share identical
  `red_lower/upper_*` and `green_lower/upper` thresholds.
- **Calibration target**: printed paper traffic light (alpha-test props) for
  current HSV tuning. Real evaluation will use **more saturated** colors —
  so tuning to the dim printed values is conservative-safe (saturated reds
  and greens will easily pass thresholds set for the dim version).
- **Architecture**: Refactor into hal/perception/control/algorithm/logs
  modules. Keep existing `camera/`, `motor/`, `sensor/` files as a legacy
  baseline — do not delete them.
- **Replay system**: Mandatory from day 1. All sensor reads and decisions are
  logged to JSONL with timestamps so logic can be debugged offline without
  the robot.

## Privacy & attribution policy (PUBLIC repo)

This repo is public. Privacy rules override convenience.

**Allowed:**
- Course name ("Basic Mobile Lab 1")
- A single copyright line referencing the user's name (already in
  CLAUDE.md / README.md headers)

**Not allowed — never commit:**
- Real names of teammates (use generic role labels: "team lead", "member A")
- Student IDs, emails, phone numbers
- TA/instructor names, profile photos, contact details
- University name, department name, lab name, building/room numbers
- Anything else in the privacy class listed under "Security (CRITICAL)" below

If a doc file contains restricted info, scrub before commit. When in doubt,
ask the user.

## Git safety rules (HARD CONSTRAINTS, set 2026-05-20)

Applies to every Claude / Codex / agent session and any
connector/plugin (Drive, Gmail, etc.) that could exfiltrate data.

1. **Never run Git commands in the home directory `~`.**
   Especially never `git add .`, `git commit`, or `git push` from `~`.

2. **Run Git only inside the intended project repo.**
   Examples: `~/teamkim-bml1`, `~/everland-kiosk`, or any path the user
   explicitly names. This project happens to live at
   `/Users/pill/Downloads/BML` — that is the repo root itself, not a
   parent folder. Do NOT `cd ..` and run git from `~/Downloads`.

3. **Verify location before any Git work.** Run `pwd` and
   `git rev-parse --show-toplevel` and confirm both point at the
   intended project repo before proceeding.

4. **Forbidden Git working directories:** `~`, `~/Downloads`, or any
   general / parent folder that may contain mixed personal files.

5. **Before pushing to a public repo, inspect the staged set.**
   `git status --short` and `git diff --cached --name-only` — make
   sure no unintended file slipped in.

6. **Never copy potentially sensitive files into a public repo.**
   Passports, IDs, raw PDFs, HEIC photos, downloaded personal documents.
   If a file is needed, use a redacted/synthetic copy.

7. **Never use `git add -f` to bypass `.gitignore`** unless the user
   explicitly asks for it.

8. **Before `git push` on a public repo with a remote, re-verify**
   the target branch and the file list one more time.

These rules also apply to outbound connectors (Drive, Gmail, etc.) —
do not stage personal artifacts to a public surface.

### Permission-request format

When asking the user to confirm a sensitive / privacy-adjacent action
(committing a photo or PDF to public repo, sharing through a connector,
adding identifying info to a doc, etc.), start the request with a
bracketed tag and keep the question to ONE line:

  `[개인정보] 이 스크린샷 안에 학번/이름 보이는데 그대로 올릴까요?`
  `[민감정보] 측정 사진에 회로 외 다른 화면 같이 찍혔는데 올려도 돼요?`
  `[보안] .env 비슷한 키 들어간 것 같은데 commit 진행할까요?`

Do NOT pad with paragraphs of context — the tag + a single yes/no
question is the whole request. Only after the user answers do we
proceed with longer reasoning if needed.

## Communication rule

After ANY change that requires the user to run something on the Pi, end the
message with a clear, copy-pasteable command block:

```
cd ~/teamkim-bml1
git pull
python <exact/path/to/file>.py
```

State which file does what and what to look for in the output. Never leave
the user guessing which command corresponds to which feature.

## Working Principle: ask for raw data, then calibrate

Claude cannot see/feel the hardware. The physical environment (motor torque,
wheel slip, sensor noise floor, lighting, traffic-light dimensions, maze wall
spacing) is unknown unless the user measures it.

**Default workflow for any tuning task:**

1. Identify what physical numbers would let you set a threshold or constant precisely
   (e.g., "HSV value of the lit red bulb at 50 cm", "ultrasonic reading 1 cm from a wall",
   "PWM duty cycle at which the car moves but doesn't slip").
2. Tell the user **which script to run, which command to type, where to point the
   sensor/camera, and what to read off the terminal** — be concrete, no hand-waving.
3. Wait for the numbers. Don't guess.
4. Plug the measured values into code with a comment recording the measurement
   conditions (date, lighting, distance), so future drift is debuggable.

Hardware capacity is generous — prefer precise, well-tuned code over conservative
defaults. Use whatever model size / sampling rate / loop frequency the Pi can handle.

## Structured logging (logs/trace.py)

All sensor reads, perception outputs, state transitions, and motor commands
should be traced via the project tracer:

```python
from logs.trace import tracer

with tracer.run("my-test"):              # opens logs/runs/<ts>_my-test.jsonl
    tracer.ultrasonic(sensor="front", raw_cm=12.4, filtered_cm=12.6, valid=True)
    tracer.camera(signal="STOP", red_area=520, green_area=8)
    tracer.decision(state="FOLLOW_WALL", action="forward", reason="L=wall F=open R=wall")
    tracer.motor(left_pwm=50, right_pwm=50, direction="forward")
```

Calls when no session is active are silent no-ops, so any module can call
`tracer.foo(...)` unconditionally.

**Reaction-delay measurement** (grading criterion) is computed offline by
pairing `camera` events with the next `motor` event in the JSONL log —
the tracer is the data source for that metric. **No metric measurement
should be done by stopwatching**; always derive from the trace.

Inspect a run: `python logs/trace.py show logs/runs/<file>.jsonl`.
Smoke-test the writer: `python logs/trace.py` (writes a demo log).

`logs/runs/` is gitignored; only the `logs/` package code is committed.

## Architecture (the live one — keep this fresh)

```
hardware_check.py    -- Pi-side wiring verification (run before anything else)
main.py              -- entry point; --dry-run works off the Pi

camera/   legacy + tools (yolo, hsv, hsv_circle, yolo_hsv, hsv_picker)
motor/    legacy + motor_calibration interactive tool
sensor/   legacy + ultrasonic_noise stats tool

hal/         ultrasonics.Ultrasonics  (3-sensor + median filter + warmup)
             motors.Motors             (drive(L,R) primitive + helpers)
perception/  traffic_light.TrafficLightDetector  (hsv_circle logic, reusable)
control/     wall_follow.WallFollowController     (smooth PD + corner anticip)
algorithm/   wall_follower_sm.WallFollowerSM       (INIT/FOLLOW/STOP@RED/PIVOT)
logs/        trace.tracer                          (JSONL session events)
docs/        test_day_checklist.md, STATUS.md
```

Update `docs/STATUS.md` when status changes; update this diagram when the
module layout changes.

## Notes for Claude

- Each script is standalone and tracks the lecture slides 1:1 — keep that mapping when editing.
- When editing `camera/yolo.py`, change `output_dir` to match the actual Pi user (`/home/<team>/Desktop/`).
- HSV thresholds (`red_lower/upper_*`, `green_lower/upper`, `min_area`) need on-site tuning under the real lighting.
- Never run YOLO and HSV at the same time — single-CPU contention.

## Security (CRITICAL — this repo is PUBLIC)

GitHub repo is public, so anything committed is world-readable forever (even after deletion — git history keeps it).

**Never commit:**
- API keys, tokens, passwords
- Wi-Fi SSID/PSK or any network credentials
- Personal info (real names beyond what's already public, phone, address, student IDs)
- Private URLs, internal hostnames, IP addresses
- `.env` files, `*.key`, `*.pem`, credential JSON

**If a secret is needed:**
1. Put it in `.env` (already gitignored — verify before adding new ignore patterns)
2. Load via `os.environ` / `python-dotenv`
3. Add an `.env.example` with dummy values so teammates know what to fill in

**Before every commit, check:**
- `git diff --staged` for accidental secrets
- No hardcoded paths containing usernames beyond the documented `/home/team2/` placeholder
- No `print(token)` / debug dumps left in code

If a secret leaks: **rotate it immediately** (changing the file later doesn't help — it's in git history).
