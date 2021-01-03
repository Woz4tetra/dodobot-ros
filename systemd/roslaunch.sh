#!/usr/bin/env bash
source /home/${USER}/ros_ws/devel/setup.bash
source /home/ben/dodobot-ros/systemd/env.sh

export ROS_HOME=/home/${USER}/.ros
roslaunch db_config dodobot.launch &
# roslaunch /home/$USER/dodobot-ros/launch/dodobot.launch &
PID=$!
wait "$PID"
