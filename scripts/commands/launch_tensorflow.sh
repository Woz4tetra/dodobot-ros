#!/bin/bash
tmux new-session -d -s tensorflow 'roslaunch db_tensorflow db_tensorflow.launch'
