BASE_DIR=$(realpath "$(dirname $0)")

SESSION=ros
IP_INTERFACE=$1
HOST_IP=$2
SSH_KEY_PATH=$3
JOYSTICK_PATH=$4

tmux has-session -t $SESSION > /dev/null

if [ $? != 0 ]; then
    tmux new -s $SESSION -d

    tmux split-window -t $SESSION:0
    tmux split-window -t $SESSION:0
    tmux split-window -t $SESSION:0
    tmux select-layout -t $SESSION:0 tiled
fi

tmux send -t $SESSION:0.0 "ssh -i $SSH_KEY_PATH ben@$HOST_IP -p 5810" ENTER 
tmux send -t $SESSION:0.0 "~/dodobot-ros/scripts/tail-session.sh" ENTER 

tmux send -t $SESSION:0.1 "ssh -i $SSH_KEY_PATH ben@$HOST_IP -p 5810" ENTER 

tmux send -t $SESSION:0.2 "source ~/dodobot-ros/scripts/set_client.sh $IP_INTERFACE $HOST_IP" ENTER 
tmux send -t $SESSION:0.2 "roslaunch db_debug_joystick db_debug_joystick.launch device:=$JOYSTICK_PATH topic_name:=joy_remote" ENTER 

tmux send -t $SESSION:0.3 "source ~/dodobot-ros/scripts/set_client.sh $IP_INTERFACE $HOST_IP" ENTER 
tmux send -t $SESSION:0.3 "rviz -d ~/dodobot-ros/db_viz/rviz/standard.rviz" ENTER 

tmux a -t $SESSION
echo "Started local session"
