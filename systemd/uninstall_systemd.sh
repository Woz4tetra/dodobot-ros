#!/usr/bin/env bash

echo "Running dodobot-ros systemd service uninstall script"

BASE_DIR=$(realpath "$(dirname $0)")

if [ "${BASE_INSTALL_DIR}" = "" ]; then
  BASE_INSTALL_DIR=~/.local/dodobot/dodobot-ros
fi

SERVICE_NAME=dodobot_ros.service
LAUNCH_SERVICE_NAME=roslaunch.service

rm -r ~/.config/systemd/user/${SERVICE_NAME}
rm -r ~/.config/systemd/user/${LAUNCH_SERVICE_NAME}

echo "Disabling systemd services"
systemctl --user daemon-reload
systemctl --user stop ${SERVICE_NAME}
systemctl --user stop ${LAUNCH_SERVICE_NAME}
loginctl enable-linger $USER
systemctl --user disable ${SERVICE_NAME}
systemctl --user disable ${LAUNCH_SERVICE_NAME}
echo "dodobot-ros systemd service uninstallation complete"
