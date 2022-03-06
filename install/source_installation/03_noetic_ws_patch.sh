#!/usr/bin/env bash

cd ~/noetic_ws
find ./src -type f -name CMakeLists.txt -exec sed -i'' -e 's/Boost REQUIRED python37/Boost REQUIRED python/g' {} +
