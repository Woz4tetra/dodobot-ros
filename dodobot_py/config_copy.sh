#!/usr/bin/env bash

echo "Running dodobot_py copy config script"

if [ "${BASE_DIR}" = "" ]; then
    BASE_DIR=$(realpath "$(dirname $0)")
fi
if [ "${SRC_DIR}" = "" ]; then
    SRC_DIR=${BASE_DIR}/dodobot_py
fi

if [ "${BASE_INSTALL_DIR}" = "" ]; then
    BASE_INSTALL_DIR=~/.local/dodobot/dodobot_py
fi

echo "Installing config files"
cp -r ${SRC_DIR}/config ${BASE_INSTALL_DIR}
echo ${BASE_INSTALL_DIR}

echo "dodobot_py copy config complete"
