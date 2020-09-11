#!/usr/bin/env bash

BASE_DIR=$(realpath "$(dirname $0)")

if ! grep -qz "`cat ${BASE_DIR}/bashrc_append.txt`" ~/.bashrc; then
    echo "Appending ros setup scripts to ~/.bashrc"
    cat ${BASE_DIR}/bashrc_append.txt | sudo tee -a ~/.bashrc > /dev/null
fi

chmod +x ${BASE_DIR}/set_master.sh

source ~/.bashrc
echo "re-open this terminal session for bashrc to go into affect"
