#!/usr/bin/env bash

echo "Running dodobot_py systemd service install script"

BASE_DIR=$(realpath "$(dirname $0)")

if [ "${BASE_INSTALL_DIR}" = "" ]; then
  BASE_INSTALL_DIR=~/.local/dodobot/dodobot_py
fi

SCRIPT_NAME=dodobot_py
SERVICE_NAME=dodobot_py.service

chmod +x ${BASE_DIR}/${SCRIPT_NAME}

BIN_INSTALL_DIR=${BASE_INSTALL_DIR}/bin
mkdir -p ${BIN_INSTALL_DIR}

echo "Copying service files"
mkdir -p ~/.config/systemd/user
cp ${BASE_DIR}/${SERVICE_NAME} ~/.config/systemd/user
cp ${BASE_DIR}/${SCRIPT_NAME} ${BIN_INSTALL_DIR}

echo "Enabling systemd services"
systemctl --user daemon-reload
systemctl --user restart ${SERVICE_NAME}
loginctl enable-linger $USER
systemctl --user enable ${SERVICE_NAME}
echo "dodobot_py systemd service installation complete"
