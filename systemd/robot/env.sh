#!/usr/bin/env bash

LOCAL_MACHINE=""
HOST_MACHINE="192.168.0.21"
# HOST_MACHINE=""

stop_time=$((SECONDS+300))

while true; do
    LOCAL_MACHINE=`ifconfig wlan0 | grep -Eo 'inet (addr:)?([0-9]*\.){3}[0-9]*' | grep -Eo '([0-9]*\.){3}[0-9]*' | grep -v '127.0.0.1'`
    if [ ! -z ${LOCAL_MACHINE} ]; then
        break
    fi
    LOCAL_MACHINE=`ifconfig wlan1 | grep -Eo 'inet (addr:)?([0-9]*\.){3}[0-9]*' | grep -Eo '([0-9]*\.){3}[0-9]*' | grep -v '127.0.0.1'`
    if [ ! -z ${LOCAL_MACHINE} ]; then
        break
    fi
    sleep 0.5
    if [ $SECONDS -gt $stop_time ]; then
        break
    fi
done

if [ ! -z ${HOST_MACHINE} ]; then
    HOST_MACHINE=${LOCAL_MACHINE}
fi

export ROS_IP=${LOCAL_MACHINE}
export ROS_MASTER_URI=http://${HOST_MACHINE}:11311

echo ${ROS_IP}
echo ${ROS_MASTER_URI}
