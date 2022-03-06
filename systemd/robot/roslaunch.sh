#!/usr/bin/env bash
echo "Starting tj2 roslaunch"

SESSION=roslaunch

tmux has-session -t $SESSION > /dev/null

if [ $? != 0 ]; then
    tmux new -s $SESSION -d
fi

tmux send -t $SESSION "source /home/${USER}/packages_ros_ws/devel/setup.bash" ENTER
tmux send -t $SESSION "source /home/${USER}/noetic_ws/install_isolated/setup.bash" ENTER
tmux send -t $SESSION "source /home/${USER}/ros_ws/devel/setup.bash" ENTER
tmux send -t $SESSION ". /usr/local/bin/env.sh 0 roslaunch" ENTER

tmux send -t $SESSION "export ROS_HOME=/home/${USER}/.ros" ENTER
tmux send -t $SESSION "export DISPLAY=:0" ENTER
tmux send -t $SESSION "roslaunch --wait db_config dodobot.launch --screen" ENTER
