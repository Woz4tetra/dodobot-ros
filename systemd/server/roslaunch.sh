#!/usr/bin/env bash
source /opt/ros/noetic/setup.bash
source /home/${USER}/ros_ws/devel/setup.bash
source /usr/local/bin/env.sh

export ROS_HOME=/home/${USER}/.ros
export DISPLAY=:0
roslaunch db_config website.launch &
PID=$!
wait "$PID"
