#!/usr/bin/env bash
BASE_DIR=$(realpath "$(dirname $0)")

packages=(
    serial
    joy
    geometry2
    teb-local-planner
    rtabmap-ros
    realsense2-camera
    vision-msgs
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
    ira-laser-tools
    pointcloud-to-laserscan
)

repos=(
    git@github.com:frc-88/navigation.git
    git@github.com:frc-88/navigation_msgs.git
)
branches=(
    noetic-devel    # git@github.com:frc-88/navigation.git
    ros1    # git@github.com:frc-88/navigation_msgs.git
)

package_list=""
for p in "${packages[@]}"; do
    package_list+="ros-noetic-$p "
done

sudo apt-get install -y $package_list

# sudo apt install -y python3-pip portaudio19-dev python3-pyaudio
# sudo pip3 install -r requirements.txt


# DEPENDENCIES_WS=$HOME/packages_ros_ws
# DEPENDENCIES_WS_SRC=${DEPENDENCIES_WS}/src

# mkdir -p ${DEPENDENCIES_WS_SRC}
# cd ${DEPENDENCIES_WS_SRC}
# len=${#repos[@]}
# for (( i=0; i<$len; i++ )); do
#     git clone --recursive ${repos[i]} --branch ${branches[i]}
# done

# cd ..
