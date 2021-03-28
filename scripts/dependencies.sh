#!/usr/bin/env bash

packages=(
joystick-drivers
geometry2
navigation
teb-local-planner
rtabmap-ros
realsense2-camera
image-geometry
vision-msgs
)

package_list=""
for p in "${packages[@]}"; do
    package_list+="ros-noetic-$p "
done

sudo apt install $package_list

sudo apt -y install llvm-9
LLVM_CONFIG=llvm-config-9 pip3 install llvmlite

sudo apt -y install portaudio19-dev python3-pyaudio
pip3 install -r requirements.txt
