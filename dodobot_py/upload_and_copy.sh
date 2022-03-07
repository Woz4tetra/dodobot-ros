#!/usr/bin/env bash

echo "Running dodobot_py upload and copy script"

if systemctl --all --user | grep -Fq 'dodobot_py'; then
    systemctl stop --user dodobot_py.service
fi

if [ "${BASE_DIR}" = "" ]; then
    BASE_DIR=$(realpath "$(dirname $0)")
fi
if [ "${SRC_DIR}" = "" ]; then
    SRC_DIR=${BASE_DIR}/dodobot_py
fi

if [ "${BASE_INSTALL_DIR}" = "" ]; then
    BASE_INSTALL_DIR=~/.local/dodobot/dodobot_py
fi
mkdir -p ${BASE_INSTALL_DIR}/logs
mkdir -p ${BASE_INSTALL_DIR}/data

echo "Installing config files"
cp -r ${SRC_DIR}/config ${BASE_INSTALL_DIR}
echo ${BASE_INSTALL_DIR}

echo "Installing python source code"
mkdir -p ${BASE_INSTALL_DIR}
rsync -a --delete ${SRC_DIR} ${BASE_INSTALL_DIR}

echo "Uploading firmware"
cd /home/$USER/dodobot-firmware/firmware/
platformio run --target upload


if systemctl --all --user | grep -Fq 'dodobot_py'; then
    systemctl start --user dodobot_py.service
fi


echo "dodobot_py upload and copy complete"
