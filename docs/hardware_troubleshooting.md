# Hardware troubleshooting — ultrasonic wiring

Concrete multimeter steps to resolve the failures `hardware_check.py`
reported. Most-likely cause first. Stop after the first fix that produces
a PASS.

## Tooling

- Digital multimeter (DMM)
- A flat object 20–50 cm in front of the sensor for testing
- The Pi powered and `hardware_check.py` ready to re-run after each step

## LEFT45 (TRIG=GPIO25, ECHO=GPIO8) — symptom: `echo never went HIGH`

The sensor never produced an echo. The chirp probably never left the
transmitter, or the return line is broken.

### Step 1. VCC power to the sensor module (most common)

- DMM: red probe on the SR04 **VCC** pin, black probe on the Pi GND.
- Expected: **5.0 V (±0.1 V)** while the Pi is powered.
- If 0 V: the wire from the SR04 VCC to the Pi 5V rail is broken or
  disconnected. **Action:** reseat / re-solder, then re-run
  `python hardware_check.py`.

### Step 2. GND continuity

- DMM in continuity / beep mode (or low-Ω): one probe on SR04 **GND**,
  one on Pi GND header.
- Expected: <1 Ω.
- If open: **Action:** restore GND wire.

### Step 3. TRIG continuity

- Pi power **OFF** while doing continuity checks.
- DMM in continuity: one probe on SR04 **TRIG** pin, one on Pi
  **GPIO 25** header (physical pin 22).
- Expected: <1 Ω.
- If open: **Action:** repair TRIG wire.

### Step 4. ECHO continuity

- Same as Step 3 but: SR04 **ECHO** pin ↔ Pi **GPIO 8** (physical pin 24).
- If open: **Action:** repair ECHO wire.

### Step 5. Sensor swap test (last resort)

- Move the FRONT SR04 module (the one that PASSed) into the LEFT slot,
  reuse the existing LEFT wiring.
- Re-run `python hardware_check.py --skip-motor`.
- If FRONT-sensor-in-LEFT-slot still FAILs → wiring is broken (revisit
  Steps 1–4 with a different multimeter).
- If FRONT-sensor-in-LEFT-slot PASSes → the original LEFT SR04 module
  is dead. **Action:** swap module.

## RIGHT45 (TRIG=GPIO7, ECHO=GPIO12) — symptom: `echo idle HIGH`

The ECHO line reads HIGH even when no measurement is in progress. The
fault is almost always GND or a short, not the sensor itself.

### Step 1. GND continuity (most common)

- DMM continuity: SR04 **GND** ↔ Pi GND header.
- Expected: <1 Ω.
- A missing GND is the #1 cause of "echo idle HIGH" — without GND the
  ECHO line floats up to the rail.
- If open: **Action:** restore GND wire. Re-run `hardware_check.py`.

### Step 2. ECHO not shorted to VCC

- DMM resistance mode, Pi **OFF** and SR04 VCC disconnected.
- Probes between SR04 **ECHO** pin and SR04 **VCC** pin.
- Expected: very high impedance (kΩ or open).
- If 0 Ω: the ECHO wire is touching VCC (often a stray strand).
  **Action:** redo the ECHO wire termination.

### Step 3. ECHO wire goes to the right pin

- Visually confirm: SR04 ECHO → Pi physical pin **32** (BCM **GPIO 12**).
- Common mis-wiring: ECHO connected to a 5V rail by accident.

### Step 4. Sensor swap test

- Move the FRONT SR04 module into the RIGHT slot.
- Re-run `python hardware_check.py --skip-motor`.
- If still HIGH idle → wiring problem (revisit Steps 1–3).
- If PASSes → original RIGHT SR04 module is dead.

## After all sensors PASS

```bash
cd ~/teamkim-bml1
git pull
python hardware_check.py
```

Three `[OK]` lines for ultrasonic + `[PASS?]` for motor visual = ready
for the sample-maze calibration day.

## Don't be surprised by

- **`--warmup`** — the first HC-SR04 ping after idle reads as an outlier
  (FRONT sample was 37 cm vs the next four at 9.8 cm). The tool now
  discards `--warmup` initial pings by default. A one-sample WARN with
  a giant `spread` you see only when warmup=0 is normal, not a wiring
  issue.
- **45° mounting angle** — the left/right sensors look at the wall at
  45°. Specular reflection off 우드락 still works, but values may read
  ~5–10 % shorter than actual normal distance. Calibrated at the
  sample maze, not pre-tuned in code.
- **Pi 3.3 V vs SR04 5 V echo** — the project intentionally skipped a
  voltage divider per the original lecture wiring. FRONT works, so the
  Pi tolerates 5 V here. If a sensor intermittently damages a GPIO over
  time, fit a 1 kΩ / 2 kΩ divider on the affected ECHO line.
