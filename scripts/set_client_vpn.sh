#!/usr/bin/env bash
LOCAL_IP=`ifconfig tun0 | grep -Eo 'inet (addr:)?([0-9]*\.){3}[0-9]*' | grep -Eo '([0-9]*\.){3}[0-9]*' | grep -v '127.0.0.1'`
# LOCAL_IP=10.8.0.1
HOST_MACHINE=10.8.0.6

if [ -z ${LOCAL_IP} ]; then
    echo "Network not configured. Not setting ROS_IP or URI"
else
    export ROS_IP=${LOCAL_IP}
    export ROS_MASTER_URI=http://${HOST_MACHINE}:11311
    # export ROS_MASTER_URI=${HOST_MACHINE}:11311

    echo ${ROS_IP}
    echo ${ROS_MASTER_URI}
fi

