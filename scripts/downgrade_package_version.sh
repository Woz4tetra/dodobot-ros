#!/usr/bin/env bash

SRC_DIR=$1

cd ${SRC_DIR}

filepattern='package.xml*'
existing='<package format=\"3\">'
replacement='<package format=\"2\">'

echo "Downgrading packages in ${SRC_DIR}"
find . -type f -name $filepattern -exec sed -i'' -e 's/$existing/$replacement/g' {} +
