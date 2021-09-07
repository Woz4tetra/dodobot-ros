#!/bin/bash

SESSION=autodock

tmux has-session -t $SESSION > /dev/null

if [ $? != 0 ]; then
    tmux new -s $SESSION -d
    tmux split-window -h -t $SESSION
    tmux split-window -v -t $SESSION
    tmux select-pane -t "$SESSION:0.0"
    tmux split-window -v -t $SESSION

else
    echo "Session is already setup"
fi

tmux select-pane -t "$SESSION:0.0"
# tmux send -t $SESSION "~/dodobot-ros/scripts/commands/start_camera.sh" ENTER
tmux send -t $SESSION "roslaunch ~/dodobot-ros/launch/auto_charge.launch map_name:=map-2021-09-01T22-50-03--023582" ENTER

# tmux select-pane -t "$SESSION:0.1"
# tmux send -t $SESSION "roslaunch db_planning db_planning.launch" ENTER

# tmux select-pane -t "$SESSION:0.2"
# tmux send -t $SESSION "roslaunch db_config move_base.launch" ENTER

tmux select-pane -t "$SESSION:0.3"
tmux send -t $SESSION "~/dodobot-ros/scripts/commands/dock.sh"

tmux a -t $SESSION
