#!/usr/bin/env bash
source /home/${USER}/packages_ros_ws/devel/setup.bash
source /home/${USER}/noetic_ws/install_isolated/setup.bash
source /home/${USER}/ros_ws/devel/setup.bash
source /home/${USER}/.local/dodobot/dodobot-ros/bin/env.sh

export ROS_HOME=/home/${USER}/.ros
export DISPLAY=:0
roslaunch db_config dodobot.launch --wait &
PID=$!
wait "$PID"
