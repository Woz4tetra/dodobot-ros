tmux new -s objects -d
tmux send -t objects "roslaunch ~/dodobot-ros/launch/objects.launch map_name:=$1" ENTER
tmux a -t objects
