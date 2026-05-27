#!/usr/bin/env bash
# Remove the boot-time autostart.
#
# Run:
#     bash scripts/uninstall_autostart.sh
#
# Stops + disables + deletes the unit file. Leaves your group memberships
# alone (gpio/video are harmless to keep).

set -euo pipefail

SERVICE_NAME=teamkim-bml1.service

echo "[uninstall] stopping + disabling $SERVICE_NAME (no error if already inactive)"
sudo systemctl disable --now "$SERVICE_NAME" || true

echo "[uninstall] removing /etc/systemd/system/$SERVICE_NAME"
sudo rm -f "/etc/systemd/system/$SERVICE_NAME"

echo "[uninstall] systemctl daemon-reload"
sudo systemctl daemon-reload

echo "[uninstall] DONE. Next boot will not auto-start the robot."
