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
BOOT_DELAY_S="${BOOT_DELAY_S:-15}"
RUN_DURATION_S="${RUN_DURATION_S:-60}"
RUN_NAME="${RUN_NAME:-boot_run}"

ts() { date -Is 2>/dev/null || date; }

echo "[run_on_boot] $(ts) repo=$REPO_DIR"
echo "[run_on_boot] $(ts) delay=${BOOT_DELAY_S}s duration=${RUN_DURATION_S}s name=$RUN_NAME"
echo "[run_on_boot] $(ts) sleeping ${BOOT_DELAY_S}s — place the car, then step back"

sleep "$BOOT_DELAY_S"

cd "$REPO_DIR"

# Use python3 explicitly (Pi OS Bookworm aliases python -> python3 only
# inside venvs).
echo "[run_on_boot] $(ts) starting main.py"
exec python3 main.py --duration "$RUN_DURATION_S" --name "$RUN_NAME"
