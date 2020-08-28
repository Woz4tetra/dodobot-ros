#!/usr/bin/env bash

echo "Running dodobot_ros install"

BASE_DIR=$(realpath "$(dirname $0)")
PARENT_DIR=$(realpath "${BASE_DIR}/..")

ROS_WS=$HOME/ros_ws/

mkdir -p ${ROS_WS}

# Base on http://wiki.ros.org/melodic/Installation/Ubuntu

# Setup your sources.list
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

# Set up your keys
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

echo "Installing ros desktop full"
sudo apt update
sudo apt install ros-melodic-desktop-full

echo "Installing ros dependencies"
sudo apt install python-rosdep python-rosinstall-generator python-wstool python-rosinstall build-essential

echo "Installing package dependencies"
pip install python-empy

echo "running 'sudo rosdep init'"
sudo rosdep init

echo "running 'rosdep update'"
rosdep update

cd ${ROS_WS}/src
# bash ${BASE_DIR}/clone_repos.sh

chmod +x ${BASE_DIR}/set_bashrc.sh
bash ${BASE_DIR}/set_bashrc.sh ${BASE_DIR}

cd ${ROS_WS}
catkin_make
