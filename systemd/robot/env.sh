#!/usr/bin/env bash

HOST_MACHINE=""

JUMP_THRESHOLD=5
TIMEOUT=${1:-0}
LOGPREFIX=${2:-log}

STOP_TIME=$((SECONDS+TIMEOUT))
PREV_TIME=$((SECONDS))
INTERFACE_NAME=wlan0

while true; do
    HOST_MACHINE=`ifconfig $INTERFACE_NAME | grep -Eo 'inet (addr:)?([0-9]*\.){3}[0-9]*' | grep -Eo '([0-9]*\.){3}[0-9]*' | grep -v '127.0.0.1'`
    if [[ $HOST_MACHINE =~ ^[0-9]{1,3}\.[0-9]{1,3}\.[0-9]{1,3}\.[0-9]{1,3}$ ]]; then
        break
    fi
    sleep 0.5
    if [[ $TIMEOUT != 0 ]]; then
        if (( $(echo "$SECONDS - $PREV_TIME > $JUMP_THRESHOLD" |bc -l) )); then  # check if time jumped like when the jetson connects to the internet
            PREV_TIME=$((SECONDS))
            STOP_TIME=$((SECONDS+TIMEOUT))
            echo "Experienced a time jump. Resetting timeout"
        fi
        if [ $SECONDS -gt $STOP_TIME ]; then
            echo "Failed to find host IP. Timed out after $STOP_TIME seconds. Current time: $SECONDS"
            break
        fi
        PREV_TIME=$((SECONDS))
    fi
done


export ROS_IP=${HOST_MACHINE}
export ROS_MASTER_URI=http://${HOST_MACHINE}:11311

echo ${ROS_IP}
echo ${ROS_MASTER_URI}

if [[ $TERM = "screen" ]] && [[ $(ps -p $PPID -o comm=) = tmux* ]]; then
    echo "Enabling tmux logging for $LOGPREFIX"
    LOGDIR=/home/${USER}/logs
    mkdir $LOGDIR 2> /dev/null
    LOGNAME="$LOGPREFIX-$(date '+%Y-%m-%dT%H-%M-%S').log"
    script -f $LOGDIR/${LOGNAME}
fi

