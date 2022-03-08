#!/usr/bin/env bash

cd ~/noetic_ws
./src/catkin/bin/catkin_make_isolated --install \
    -DCMAKE_BUILD_TYPE=Release \
    -DPYTHON_EXECUTABLE=/usr/bin/python3
