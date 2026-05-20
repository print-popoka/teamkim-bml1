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
├── motor/
│   └── motor.py        # L298N two-motor PWM control
└── sensor/
    └── ultrasonic.py   # HC-SR04 distance measurement
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
```
