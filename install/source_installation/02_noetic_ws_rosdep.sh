#!/usr/bin/env bash

cd ~/noetic_ws
rosdep install --from-paths ./src --ignore-packages-from-source --rosdistro noetic -y -r
