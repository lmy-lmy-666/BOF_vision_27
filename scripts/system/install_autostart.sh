#!/usr/bin/env bash
# Run on the robot computer: installs udev rules (sudo) and the systemd user service.
set -euo pipefail
cd -P "$(dirname "${BASH_SOURCE[0]}")/../.."
root="$PWD"
sudo install -m 644 scripts/system/99-bof-vision.rules /etc/udev/rules.d/99-bof-vision.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
mkdir -p ~/.config/systemd/user
sed "s|@ROOT@|$root|g" scripts/system/bof-vision.service > ~/.config/systemd/user/bof-vision.service
systemctl --user daemon-reload
systemctl --user enable bof-vision.service
# Start at boot without an interactive login.
sudo loginctl enable-linger "$USER"
echo "Installed. Start now: systemctl --user start bof-vision; logs: journalctl --user -u bof-vision -f"
echo "Disable: systemctl --user disable --now bof-vision"
