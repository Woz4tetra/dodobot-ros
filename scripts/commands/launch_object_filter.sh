#!/bin/bash
tmux new-session -d -s object_filter 'roslaunch db_object_filter db_object_filter.launch'
