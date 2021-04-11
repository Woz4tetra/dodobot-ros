tmux new -s objects -d
tmux send -t objects "roslaunch ~/dodobot-ros/launch/objects.launch" ENTER
tmux a -t objects
