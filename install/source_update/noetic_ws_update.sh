#!/usr/bin/env bash

cd ~/noetic_ws
mv -i noetic-desktop.rosinstall noetic-desktop.rosinstall.old

rosinstall_generator desktop --rosdistro noetic --deps --tar > noetic-desktop.rosinstall
diff -u noetic-desktop.rosinstall noetic-desktop.rosinstall.old

vcs import --input noetic-desktop.rosinstall ./src

