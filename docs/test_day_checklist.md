# Sample-maze test day checklist

Goal: walk into the sample maze with a known plan, walk out with all the
numbers we need to set the calibration constants in code.

## Bring

- [ ] Laptop / phone for typing measurements into Claude
- [ ] **Ruler** (30cm or longer)
- [ ] **Stopwatch** (phone is fine)
- [ ] **Protractor** (phone protractor app works)
- [ ] **Multimeter** (L298N motor stall-current check)
- [ ] Masking tape (mark start positions)
- [ ] Chalk / marker (turn-test reference direction)
- [ ] Spare batteries / fresh 12V cell — calibration drifts with voltage
- [ ] Raspberry Pi with the latest code pulled
- [ ] Phone camera (record video — invaluable for post-analysis)

## Pre-check on arrival

```bash
cd ~/teamkim-bml1
git pull
python hardware_check.py
```

All three ultrasonic checks should report **PASS** (or a warm-up WARN);
if any FAIL, fix wiring before starting calibration. Relay specific
failures to the hardware team (CLAUDE.md "role boundary").

## Order of measurements (~1.5 hours total)

### 30 min — Ultrasonic noise + 우드락 wall behaviour

For each sensor and each true distance:

```bash
# FRONT
python sensor/ultrasonic_noise.py --true 10 --quiet
python sensor/ultrasonic_noise.py --true 30 --quiet
python sensor/ultrasonic_noise.py --true 50 --quiet

# LEFT45
python sensor/ultrasonic_noise.py --trig 25 --echo 8 --true 10 --quiet
python sensor/ultrasonic_noise.py --trig 25 --echo 8 --true 30 --quiet

# RIGHT45
python sensor/ultrasonic_noise.py --trig 7  --echo 12 --true 10 --quiet
python sensor/ultrasonic_noise.py --trig 7  --echo 12 --true 30 --quiet
```

Send the stats blocks back — we set median window and outlier thresholds.

### 45 min — Motor calibration

```bash
python motor/motor_calibration.py --trials 3
```

Runs four tests in order:
1. **Min start PWM** (left, then right)
2. **Speed table** (PWM 30/50/70/90, forward 1 s)
3. **Straight drift** (PWM 50, forward 2 s)
4. **90 deg turn time** (PWM 50, in-place right pivot)

The final printed block is the canonical calibration. Paste it back — I
commit it into `hal/motors.py` as named constants.

**Also**: multimeter inline with motor supply, force a wheel to stall,
read current. Need it under 2A/channel or we order a second L298N.

### 15 min — Camera HSV re-verify

```bash
python camera/hsv_picker.py
```

Click red ON, red OFF, green ON, green OFF, background — 3 times each, on
the **actual sample-maze traffic-light print**. If values match our
existing calibration within ~20 on S/V, no change needed.

### 15 min — Misc capture

- [ ] Photos of each corner type (T, +, dead-end, narrowing section)
- [ ] Photo of the start position with the car placed
- [ ] Note lighting type (fluorescent / LED / sunlight)
- [ ] Note corridor widest + narrowest measurements
- [ ] If motors+ultrasonic look OK, run a 30 s smoke test:
      `python main.py --duration 30 --name sample_dryrun`
      Even with placeholder constants, the trace tells us a lot.

## After the test, send back

1. `python hardware_check.py` output
2. Ultrasonic stats blocks per sensor (or the JSONL log)
3. Motor calibration final block
4. Camera HSV samples (or "no change")
5. Multimeter stall-current reading
6. Photos / video
7. Notes on anything weird

I bake the numbers into the code and push.
