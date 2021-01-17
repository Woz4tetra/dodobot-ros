#!/usr/bin/env bash

packages=(
serial
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
    package_list+="ros-melodic-$p "
done

sudo apt install $package_list

# see: https://stackoverflow.com/questions/48690984/portaudio-h-no-such-file-or-directory
sudo apt-get install portaudio19-dev python-pyaudio

pip install -r requirements.txt