#!/usr/bin/env bash
source /opt/ros/noetic/setup.bash
source /home/${USER}/ros_ws/devel/setup.bash
source /usr/local/bin/env.sh
roscore & while ! echo exit | nc localhost 11311 > /dev/null; do sleep 1; done
