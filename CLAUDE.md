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

- **Algorithm**: Start with refined Pledge (closed-loop turns verified by
  ultrasonic). Swap to Tremaux as a fallback only if Pledge counter proves
  unreliable in real testing. Algorithm choice may evolve during coding —
  reconfirm with user before pivoting.
- **Ultrasonic layout**: 3 sensors — front-center, front-left (~45°),
  front-right (~45°), each with independent TRIG/ECHO (pin map above).
  Enables wall-following + junction detection.
- **Motor wiring**: 4 DC motors, paired left/right. Specific wiring (single
  L298N parallel vs 2x L298N) deferred until first integration test.
- **Traffic light**: official requirement is Red→STOP / Green→GO only.
  Yellow→SLOW is implemented (alpha test had yellow) but its presence in the
  real test is unconfirmed. Keep the code behind a config flag
  (`ENABLE_YELLOW`) so it can be toggled without edits. Decide later.
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
