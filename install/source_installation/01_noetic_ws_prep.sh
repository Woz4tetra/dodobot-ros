#!/usr/bin/env bash

mkdir -p ~/noetic_ws/src
cd ~/noetic_ws

echo "Downloading noetic package lists"
rosinstall_generator desktop --rosdistro noetic --deps --tar > noetic-desktop.rosinstall
echo "done!"
echo "Parsing rosinstall file"
vcs import --input noetic-desktop.rosinstall ./src

