#!/usr/bin/env bash
source /home/ben/noetic_ws/install_isolated/setup.bash
source /home/ben/.local/dodobot/dodobot-ros/bin/env.sh
roscore & while ! echo exit | nc localhost 11311 > /dev/null; do sleep 1; done
