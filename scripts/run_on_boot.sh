#!/usr/bin/env bash
# Auto-start wrapper for the maze robot main loop.
#
# Triggered by the systemd unit `teamkim-bml1.service` at every boot.
# Sleeps a few seconds (so you can place the car after connecting the
# battery), then runs main.py for a fixed duration. Trace lands in
# logs/runs/<ts>_<name>.jsonl on the SD card; retrieve later via SSH
# or by reading the SD card on a laptop.
#
# All tunables come from environment variables — override them with
# `sudo systemctl edit teamkim-bml1` or via /etc/default/teamkim-bml1.

set -euo pipefail

REPO_DIR="${REPO_DIR:-/home/team2/teamkim-bml1}"
BOOT_DELAY_S="${BOOT_DELAY_S:-20}"
RUN_DURATION_S="${RUN_DURATION_S:-300}"
RUN_NAME="${RUN_NAME:-boot_run}"
# Which entrypoint to run. main.py is the full wall-following maze loop.
# integration_test.py remains available as an explicit override only.
RUN_SCRIPT="${RUN_SCRIPT:-main.py}"

ts() { date -Is 2>/dev/null || date; }

# Dev-mode guard: if a monitor (HDMI) is connected, assume we are at the desk
# for development (not on battery in the field) and skip the autorun, so the
# car cannot drive off while you work. `sudo systemctl disable` is still the
# guaranteed off-switch; this is a convenience on top of it.
#   - Force the run even with a monitor attached: FORCE_AUTORUN=1
#   - Detection path is overridable so it can be tested off the Pi: DRM_STATUS_GLOB
DRM_STATUS_GLOB="${DRM_STATUS_GLOB:-/sys/class/drm/card*-HDMI*/status}"
# shellcheck disable=SC2086  # intentional glob expansion of the status path
if [ "${FORCE_AUTORUN:-0}" != "1" ] && grep -qsx connected $DRM_STATUS_GLOB; then
    echo "[run_on_boot] $(ts) HDMI display connected -> dev mode, skipping autorun"
    echo "[run_on_boot] $(ts) (run on battery with no monitor, or set FORCE_AUTORUN=1 to force)"
    exit 0
fi

echo "[run_on_boot] $(ts) repo=$REPO_DIR"
echo "[run_on_boot] $(ts) delay=${BOOT_DELAY_S}s duration=${RUN_DURATION_S}s name=$RUN_NAME"
echo "[run_on_boot] $(ts) sleeping ${BOOT_DELAY_S}s — place the car, then step back"

sleep "$BOOT_DELAY_S"

cd "$REPO_DIR"

# Use python3 explicitly (Pi OS Bookworm aliases python -> python3 only
# inside venvs).
echo "[run_on_boot] $(ts) starting $RUN_SCRIPT"
exec python3 "$RUN_SCRIPT" --duration "$RUN_DURATION_S" --name "$RUN_NAME"
