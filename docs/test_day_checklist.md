# Two-phase calibration plan

Most calibrations don't depend on the maze (motor PWM, ultrasonic noise,
HSV thresholds). Doing them in advance turns sample-maze day from a
1.5-hour scramble into a 30-minute verify-and-tune session — and the
robot arrives already working.

---

## Phase A — pre-maze calibration (anytime, anywhere)

Run this in the lab / dorm / any flat-floor space with a few flat
surfaces. Prerequisite: hardware team has fixed LEFT45 and RIGHT45 so
that `python hardware_check.py` reports three `[OK]` ultrasonics.

### Bring

- [ ] Ruler 30 cm+
- [ ] Stopwatch (phone)
- [ ] Protractor (phone app)
- [ ] Multimeter (for L298N stall current)
- [ ] Masking tape
- [ ] Pi + 12V battery (fully charged)

### Step 1 — sanity

```bash
cd ~/teamkim-bml1
git pull
python hardware_check.py
```

If anything FAILs → fix wiring first (`docs/hardware_troubleshooting.md`).

### Step 2 — ultrasonic noise (10 min)

Aim the sensor at a flat object at known distances:

```bash
# FRONT
python sensor/ultrasonic_noise.py --true 10  --quiet
python sensor/ultrasonic_noise.py --true 30  --quiet
python sensor/ultrasonic_noise.py --true 50  --quiet
python sensor/ultrasonic_noise.py --true 100 --quiet

# LEFT45
python sensor/ultrasonic_noise.py --trig 25 --echo 8 --true 10 --quiet
python sensor/ultrasonic_noise.py --trig 25 --echo 8 --true 30 --quiet

# RIGHT45
python sensor/ultrasonic_noise.py --trig 7  --echo 12 --true 10 --quiet
python sensor/ultrasonic_noise.py --trig 7  --echo 12 --true 30 --quiet
```

Paste each stats block back to Claude.

### Step 3 — motor calibration (45 min)

```bash
python motor/motor_calibration.py --trials 3
```

Drives 4 tests, you measure with ruler/protractor, type in numbers. Paste the final CALIBRATION block back to Claude.

### Step 4 — multimeter stall current

While a wheel is forced to stall, measure current inline with the 12 V supply. Must stay under 2 A/channel (single L298N) or we order a second L298N.

### Step 5 — Claude bakes constants into code, pushes

After steps 2–4 you'll see updated `hal/motors.py` and `hal/ultrasonics.py` with measured values committed.

### Step 6 — first drive in the lab

```bash
git pull
python main.py --duration 20 --name lab_first
python logs/replay.py "$(ls -t logs/runs/*.jsonl | head -1)" --latency
```

Watch the car. Send the trace back. Claude tunes the PD gains based on what actually happened. Iterate 2–3 times until the lab drive looks smooth.

**End of Phase A:** the car drives correctly in a controlled environment. We're ready for the actual maze.

---

## Phase B — sample-maze day (30 min)

### Bring

- [ ] Everything from Phase A, **plus** `hsv_picker.py`-ready Pi
- [ ] Phone camera (record video)
- [ ] Notebook

### Step 1 — sanity (2 min)

```bash
cd ~/teamkim-bml1
git pull
python hardware_check.py
```

### Step 2 — 우드락 wall verification (5 min)

Place the car ~30 cm from a 우드락 wall on each side; quick re-measure:

```bash
python sensor/ultrasonic_noise.py --true 30 --quiet
python sensor/ultrasonic_noise.py --trig 25 --echo 8 --true 30 --quiet
python sensor/ultrasonic_noise.py --trig 7  --echo 12 --true 30 --quiet
```

If bias > 2 cm vs Phase A results, the 우드락 surface reads short on
the 45° sensors — Claude updates the per-sensor bias constants.

### Step 3 — HSV re-verify (5 min, only if lighting looks different)

```bash
python camera/hsv_picker.py
```

Click red ON / green ON / red OFF / green OFF / background. If values
fall outside the locked thresholds, paste back; Claude retunes.

### Step 4 — actual drive + tune (15 min)

```bash
python main.py --duration 60 --name maze_run_01
```

Watch the car. Send back:
- `logs/runs/maze_run_01_*.jsonl` (or describe what happened)
- Photo / video if anything unusual
- Specific complaints ("hit left wall at corner 2", "got stuck spinning at dead-end")

Claude reads the trace + adjusts `control/wall_follow.py` constants → pushes → you `git pull` and retry. Aim for 2–3 iteration cycles in the 15 minutes.

### Step 5 — record the final run

```bash
python main.py --duration 120 --name maze_final
```

Phone-record the drive for the evaluation video.

---

## After both phases

Send back:
1. All ultrasonic stats blocks (Phase A + Phase B re-verify)
2. Motor calibration block (Phase A)
3. Multimeter stall current
4. HSV samples (if retuned in Phase B)
5. Final maze trace + video

Claude assembles the final-presentation analysis (completion time,
collision count, reaction delay) from the trace files.
