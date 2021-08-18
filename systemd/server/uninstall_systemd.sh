#!/usr/bin/env bash
if [ "$EUID" -ne 0 ]
    then echo "Please run as root"
    exit
fi

echo "Running Diff-Swerve-ROS systemd service uninstall script"

BASE_DIR=$(realpath "$(dirname $0)")

if [ "${BASE_INSTALL_DIR}" = "" ]; then
  BASE_INSTALL_DIR=/usr/local
fi

SERVICE_NAME=roscore.service
LAUNCH_SERVICE_NAME=roslaunch.service

SCRIPT_NAME=roscore.sh
ENV_SCRIPT_NAME=env.sh
SERVICE_NAME=roscore.service

BIN_INSTALL_DIR=${BASE_INSTALL_DIR}/bin

SERVICE_ROOT_DIR=/etc/systemd/system/

rm ${SERVICE_ROOT_DIR}/${SERVICE_NAME}
rm ${SERVICE_ROOT_DIR}/${LAUNCH_SERVICE_NAME}

rm ${BIN_INSTALL_DIR}/${SCRIPT_NAME}
rm ${BIN_INSTALL_DIR}/${ENV_SCRIPT_NAME}
rm ${BIN_INSTALL_DIR}/${LAUNCH_SCRIPT_NAME}


echo "Disabling systemd services"
systemctl daemon-reload
systemctl stop ${SERVICE_NAME}
systemctl stop ${LAUNCH_SERVICE_NAME}

systemctl disable ${SERVICE_NAME}
systemctl disable ${LAUNCH_SERVICE_NAME}

echo "Diff-Swerve-ROS systemd service uninstallation complete"
