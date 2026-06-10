# Headless boot — battery-only sample maze workflow

No keyboard, no mouse, no monitor. Plug battery → wait → car drives → trace
lands on the SD card. Power-cycle = next run.

## One-time setup on the Pi (do this at home / dorm with keyboard)

```bash
cd ~/teamkim-bml1
git pull
bash scripts/install_autostart.sh
sudo reboot                # required so group membership (gpio + video) sticks
```

After reboot, every power-on will:
1. Wait `BOOT_DELAY_S` seconds (default 20) — place the car, step away
2. Run `python3 main.py --duration RUN_DURATION_S --name RUN_NAME`
   (defaults: 300 seconds, name `boot_run`)
3. Trace lands in `~/teamkim-bml1/logs/runs/<timestamp>_boot_run.jsonl`
4. Pi stays powered after run completes — pull battery whenever

> **Dev-mode guard.** If a monitor (HDMI) is connected, the autorun is skipped
> so the car can't drive off while you work at the desk. Boot on battery with
> **no monitor** for it to actually run. Force a run with a monitor attached
> via `FORCE_AUTORUN=1`.

## Stop it auto-driving while you develop (monitor/keyboard attached)

The HDMI guard above covers the common case, but the guaranteed control is the
systemd service itself:

```bash
sudo systemctl disable --now teamkim-bml1.service   # OFF: never autorun on boot
sudo systemctl enable teamkim-bml1.service          # ON:  autorun on next boot
systemctl is-enabled teamkim-bml1.service           # check -> enabled / disabled
sudo systemctl stop teamkim-bml1.service            # stop a run already in progress
```

Recommended: keep it **disabled** while developing, and **enable** it only right
before a battery field run.

## Tuning run duration / name at the sample maze

You only need this if you want a longer drive or a per-trial name:

```bash
sudo systemctl edit teamkim-bml1
# editor opens — add:
#   [Service]
#   Environment=RUN_DURATION_S=90
#   Environment=RUN_NAME=maze_lap_03
# save + exit
```

Then power-cycle the Pi for the next run.

## Retrieving traces after a run

Three options, easiest first:

**A. SD card swap** (no network needed)
- Power off Pi cleanly: `sudo shutdown -h now` (if you have SSH) or just
  wait for the run to end and pull the battery
- Eject SD card, plug into laptop
- Copy `teamkim-bml1/logs/runs/*.jsonl`

**B. SSH from phone hotspot** (if Pi is configured to join the hotspot)
- Phone hotspot ON, SSID/PSK already in `wpa_supplicant.conf`
- After Pi boots, find IP: `nmap -sn 192.168.x.0/24` or check phone's hotspot client list
- `scp team2@<ip>:teamkim-bml1/logs/runs/*.jsonl ./`

**C. ngrok / cloudflared tunnel** (if you really need remote access mid-run)
- Pre-configure on Pi, runs in background. Out of scope for this doc.

## "Help, the car is misbehaving and I can't stop it"

- **Pull the battery.** That's the panic stop. SD-card-corruption risk is
  real but small (the tracer flushes per line); worth it vs. a crash.
- Better: power down cleanly when you have time:
  - SSH in from phone (`ssh team2@<ip>`)
  - `sudo systemctl stop teamkim-bml1` — kills the current run
  - `sudo systemctl disable teamkim-bml1` — prevents next boot from running it
  - `sudo shutdown -h now`

## "I want to disable autostart for one run only"

Easiest: don't power-cycle. Stay powered, SSH in, run things manually.

If you must power-cycle without autostart, the cleanest way is to disable
once and re-enable later:

```bash
sudo systemctl disable teamkim-bml1   # next boot will NOT run
# ... do your thing ...
sudo systemctl enable  teamkim-bml1   # next boot WILL run again
```

## What this DOES NOT do

The autostart runs `main.py` only — the actual drive logic. It does **not**
run the interactive calibration tools:

- `motor/motor_calibration.py` — needs typed input per trial
- `sensor/ultrasonic_noise.py` per-distance series — needs you to reposition
  the reference wall between runs

For those, do them at home (Phase A) with keyboard attached, or SSH from
phone at the sample maze.

## Verifying the autostart without rebooting

```bash
# Force the wrapper script directly — bypasses systemd, useful for debugging
bash scripts/run_on_boot.sh

# Or trigger the unit manually (same as a fresh boot would do)
sudo systemctl start teamkim-bml1
journalctl -u teamkim-bml1 -f      # follow log live; Ctrl+C to exit
```
