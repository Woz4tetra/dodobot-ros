#!/usr/bin/env bash

tmux new -s noetic_ws -d
tmux send -t noetic_ws "cd ~/noetic_ws && \
    ./src/catkin/bin/catkin_make_isolated --install \
        -DCMAKE_BUILD_TYPE=Release \
        -DPYTHON_EXECUTABLE=/usr/bin/python3" ENTER
tmux a -t noetic_ws
