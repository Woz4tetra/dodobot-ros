tmux new -s filter -d
tmux send -t filter "roslaunch db_object_filter db_object_filter.launch" ENTER
tmux a -t filter
