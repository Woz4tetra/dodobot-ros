tmux new -s planning -d
tmux send -t planning "roslaunch db_planning db_planning.launch" ENTER
tmux a -t planning
