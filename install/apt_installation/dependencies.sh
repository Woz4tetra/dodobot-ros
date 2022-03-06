#!/usr/bin/env bash

packages=(
geometry2
navigation
teb-local-planner
rtabmap-ros
realsense2-camera
vision-msgs
move-base-flex
perception-pcl
octomap-msgs
pcl-msgs
apriltag-ros
costmap-converter
image-geometry
image-pipeline
image-common
laser-filters
robot-localization
gmapping
amcl
usb-cam
image-transport-plugins
rosbag-snapshot
twist-mux
)

package_list=""
for p in "${packages[@]}"; do
    package_list+="ros-noetic-$p "
done

sudo apt-get install -y $package_list

sudo apt install -y python3-pip
sudo pip3 install -r requirements.txt

