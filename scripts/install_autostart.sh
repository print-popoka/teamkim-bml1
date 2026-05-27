#!/usr/bin/env bash
# One-shot installer for the boot-time autostart on the Pi.
#
# Run ONCE after cloning the repo:
#     bash scripts/install_autostart.sh
#
# Effect:
#   - Adds your user to the gpio + video groups (required by RPi.GPIO + picamera2)
#   - Copies scripts/teamkim-bml1.service into /etc/systemd/system/
#   - Enables it so it starts at every boot
#
# Idempotent — safe to re-run.

set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
SERVICE_NAME=teamkim-bml1.service
UNIT_SRC="$REPO_DIR/scripts/$SERVICE_NAME"
UNIT_DST="/etc/systemd/system/$SERVICE_NAME"

if [[ ! -f "$UNIT_SRC" ]]; then
    echo "[install] ERROR: $UNIT_SRC not found" >&2
    exit 1
fi

echo "[install] repo : $REPO_DIR"
echo "[install] user : $(id -un)"

echo "[install] adding $(id -un) to gpio + video groups..."
sudo usermod -a -G gpio,video "$(id -un)" || true

echo "[install] copying $UNIT_SRC -> $UNIT_DST"
sudo cp "$UNIT_SRC" "$UNIT_DST"

echo "[install] systemctl daemon-reload + enable"
sudo systemctl daemon-reload
sudo systemctl enable "$SERVICE_NAME"

echo
echo "[install] DONE."
echo
echo "Every boot from now on:"
echo "  1. waits BOOT_DELAY_S (default 15) seconds"
echo "  2. runs: python3 main.py --duration RUN_DURATION_S --name RUN_NAME"
echo "  3. writes trace to logs/runs/<ts>_RUN_NAME.jsonl"
echo
echo "Useful commands on the Pi:"
echo "  Edit tunables (delay, duration, name):"
echo "    sudo systemctl edit $SERVICE_NAME"
echo "    # then add e.g.:"
echo "    #   [Service]"
echo "    #   Environment=RUN_DURATION_S=30"
echo "    #   Environment=RUN_NAME=maze_lap_01"
echo
echo "  Disable temporarily (skip autostart on next boot):"
echo "    sudo systemctl disable --now $SERVICE_NAME"
echo
echo "  Re-enable:"
echo "    sudo systemctl enable --now $SERVICE_NAME"
echo
echo "  View this boot's log:"
echo "    journalctl -u $SERVICE_NAME -b"
echo
echo "  View previous boot's log:"
echo "    journalctl -u $SERVICE_NAME -b -1"
echo
echo "  Test the wrapper now without rebooting (no autostart involvement):"
echo "    bash scripts/run_on_boot.sh"
echo
echo "[install] NOTE: group changes (gpio/video) take effect after the"
echo "          next login. If this is a fresh install, reboot once now:"
echo "            sudo reboot"
