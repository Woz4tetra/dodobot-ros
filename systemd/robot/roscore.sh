#!/usr/bin/env bash
source /home/${USER}/noetic_ws/install_isolated/setup.bash
. /usr/local/bin/env.sh 0 roscore
roscore & while ! echo exit | nc localhost 11311 > /dev/null; do sleep 1; done
