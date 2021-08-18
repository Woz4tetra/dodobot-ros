#!/usr/bin/env bash
if [ "$EUID" -ne 0 ]
    then echo "Please run as root"
    exit
fi

echo "Running dodobot-ros systemd service install script"

BASE_DIR=$(realpath "$(dirname $0)")

if [ "${BASE_INSTALL_DIR}" = "" ]; then
    BASE_INSTALL_DIR=~/.local/dodobot/dodobot-ros
fi

SCRIPT_NAME=roscore.sh
ENV_SCRIPT_NAME=env.sh
SERVICE_NAME=roscore.service

LAUNCH_SCRIPT_NAME=roslaunch.sh
LAUNCH_SERVICE_NAME=roslaunch.service

chmod +x ${BASE_DIR}/${SCRIPT_NAME}
chmod +x ${BASE_DIR}/${ENV_SCRIPT_NAME}
chmod +x ${BASE_DIR}/${LAUNCH_SCRIPT_NAME}

BIN_INSTALL_DIR=${BASE_INSTALL_DIR}/bin
mkdir -p ${BIN_INSTALL_DIR}

echo "Copying service files"
SERVICE_ROOT_DIR=/etc/systemd/system/
mkdir -p ${SERVICE_ROOT_DIR}
cp ${BASE_DIR}/${SERVICE_NAME} ${SERVICE_ROOT_DIR}
cp ${BASE_DIR}/${LAUNCH_SERVICE_NAME} ${SERVICE_ROOT_DIR}

cp ${BASE_DIR}/${SCRIPT_NAME} ${BIN_INSTALL_DIR}
cp ${BASE_DIR}/${ENV_SCRIPT_NAME} ${BIN_INSTALL_DIR}
cp ${BASE_DIR}/${LAUNCH_SCRIPT_NAME} ${BIN_INSTALL_DIR}

echo "Enabling systemd services"
systemctl daemon-reload
loginctl enable-linger $USER
# systemctl enable ${SERVICE_NAME}
systemctl enable ${LAUNCH_SERVICE_NAME}

# systemctl restart ${SERVICE_NAME}
systemctl restart ${LAUNCH_SERVICE_NAME}

echo "dodobot-ros systemd service installation complete"
