#!/usr/bin/env bash

echo "Running dodobot_py uninstall script"

BASE_DIR=$(realpath "$(dirname $0)")

if [ "${BASE_INSTALL_DIR}" = "" ]; then
  BASE_INSTALL_DIR=~/.local/dodobot/dodobot_py
fi
rm -r ${BASE_INSTALL_DIR}
