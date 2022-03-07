#!/usr/bin/env bash

echo "Running dodobot_py systemd service uninstall script"

BASE_DIR=$(realpath "$(dirname $0)")

if [ "${BASE_INSTALL_DIR}" = "" ]; then
  BASE_INSTALL_DIR=~/.local/dodobot/dodobot-ros
fi

SERVICE_NAME=dodobot_py.service

rm -r ~/.config/systemd/user/${SERVICE_NAME}

echo "Disabling systemd services"
systemctl --user daemon-reload
systemctl --user stop ${SERVICE_NAME}
loginctl enable-linger $USER
systemctl --user disable ${SERVICE_NAME}
echo "dodobot_py systemd service uninstallation complete"
