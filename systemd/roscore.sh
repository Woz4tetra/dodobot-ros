#!/usr/bin/env bash
source /opt/ros/noetic/setup.sh
source /home/ben/.local/dodobot/dodobot-ros/bin/env.sh
roscore & while ! echo exit | nc localhost 11311 > /dev/null; do sleep 1; done
