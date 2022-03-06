#!/usr/bin/env bash

tmux new -s noetic_ws -d
tmux send -t noetic_ws "cd ~/noetic_ws && \
    rosdep install --from-paths ./src --ignore-packages-from-source --rosdistro noetic -y -r" ENTER
tmux a -t noetic_ws
