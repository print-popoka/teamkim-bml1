### Copyright © Hyunjung Kim. All rights reserved.

# teamkim-bml1

Basic Mobile Lab 1 — Teamkim 팀.

## Hardware

- Raspberry Pi 4B rev 1.5 (BCM2711, 4GB RAM, MicroSD)
- Raspberry Pi Camera Rev 1.3 (CSI)
- L298N motor driver + 2x DC motors
- HC-SR04 ultrasonic sensor

## GPIO (BCM)

| Function | Pin |
|---|---|
| Motor A IN1 / IN2 | GPIO 17, 27 |
| Motor B IN3 / IN4 | GPIO 22, 5 |
| ENA / ENB (PWM) | GPIO 18, 19 |
| Ultrasonic TRIG | GPIO 23 |
| Ultrasonic ECHO | GPIO 24 |

## Constraints

- CPU only — no GPU. YOLOv8n is the largest model usable.
- Pip installs **must** use `--break-system-packages`.
- Set `TMPDIR=/home/team2` before installing (avoids /tmp space issues).
- **Do not run YOLO and HSV simultaneously** — single-CPU contention will stall both.

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
