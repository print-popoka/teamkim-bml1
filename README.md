# Copyright © Hyunjung Kim. All rights reserved.

## teamkim-bml1

## Structure

```
.
├── CLAUDE.md           # Project context for Claude Code
├── README.md
├── camera/
│   ├── yolo.py         # YOLOv8n object detection
│   ├── hsv.py          # HSV traffic-light detection
│   └── hsv_circle.py   # HSV + circularity filter
├── algorithm/
│   ├── maze_logic.py   # pure maze/demo decision logic
│   └── simulate.py     # local logic simulation
├── motor/
│   ├── motor.py        # L298N two-motor PWM control
│   └── motor_calibration.py
├── sensor/
│   └── ultrasonic.py   # HC-SR04 distance measurement
└── tests/
    └── test_maze_logic.py
```

## Hardware

| Component | Detail |
|---|---|
| SBC | Raspberry Pi 4B rev 1.5 (BCM2711, 4GB) |
| Camera | Raspberry Pi Camera Rev 1.3 (CSI) |
| Motor driver | L298N |
| Sensor | HC-SR04 ultrasonic |

(Raspberry Pi + Camera + L298N motors + HC-SR04). 

## Wiring (BCM numbering)

- Motor A: IN1=17, IN2=27, ENA=18
- Motor B: IN3=22, IN4=5,  ENB=19
- Ultrasonic: TRIG=23, ECHO=24

See [CLAUDE.md](CLAUDE.md) for install steps and constraints (CPU-only, `--break-system-packages`, no simultaneous YOLO+HSV).

## Quick start

```bash
rpicam-hello -t 3000
python camera/yolo.py
python camera/hsv.py
python motor/motor.py
python sensor/ultrasonic.py
python hardware_check.py         # motor + 3x ultrasonic assembly check
python algorithm/simulate.py     # local logic simulation, no hardware needed
python -m unittest tests/test_maze_logic.py
```

## Performance-oriented runtime defaults

- `camera/hsv_circle.py` and `camera/hsv.py` default to `320x240`, top ROI,
  and throttled logging.
- `camera/yolo.py` defaults to `320x240` and saves only sampled frames.
- `camera/yolo_hsv.py` defaults to running YOLO every 4 frames and reusing the
  last decision between YOLO passes.
- Ultrasonic scripts use shorter maze-oriented echo timeouts by default.
