#!/usr/bin/env bash

echo "Running tj2_ros systemd service copy script"

BASE_DIR=$(realpath "$(dirname $0)")

if [ "${BASE_INSTALL_DIR}" = "" ]; then
    BASE_INSTALL_DIR=/usr/local
fi

SCRIPT_NAME=roscore.sh
ENV_SCRIPT_NAME=env.sh
LAUNCH_SCRIPT_NAME=roslaunch.sh

chmod +x ${BASE_DIR}/${SCRIPT_NAME}
chmod +x ${BASE_DIR}/${ENV_SCRIPT_NAME}
chmod +x ${BASE_DIR}/${LAUNCH_SCRIPT_NAME}

BIN_INSTALL_DIR=${BASE_INSTALL_DIR}/bin
mkdir -p ${BIN_INSTALL_DIR}

cp ${BASE_DIR}/${SCRIPT_NAME} ${BIN_INSTALL_DIR}
cp ${BASE_DIR}/${ENV_SCRIPT_NAME} ${BIN_INSTALL_DIR}
cp ${BASE_DIR}/${LAUNCH_SCRIPT_NAME} ${BIN_INSTALL_DIR}

echo "tj2_ros systemd service copy complete"
