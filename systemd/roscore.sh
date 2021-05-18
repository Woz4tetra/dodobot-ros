#!/usr/bin/env bash
source /home/${USER}/noetic_ws/install_isolated/setup.bash
source /home/${USER}/.local/dodobot/dodobot-ros/bin/env.sh
roscore & while ! echo exit | nc localhost 11311 > /dev/null; do sleep 1; done
