#!/usr/bin/env bash

echo "Running dodobot_ros install"

BASE_DIR=$(realpath "$(dirname $0)")

${BASE_DIR}/build_packages_ws.sh
success=$?
if [[ $success -eq 1 ]];
then
    exit 1
fi

chmod +x ${BASE_DIR}/set_bashrc.sh
bash ${BASE_DIR}/set_bashrc.sh ${BASE_DIR}
source $HOME/.bashrc

ROS_WS=$HOME/ros_ws/
ROS_WS_SRC=${ROS_WS}/src

mkdir -p ${ROS_WS_SRC}
cd ${ROS_WS}
catkin_make

# disable serial console on /dev/ttyTHS1
sudo systemctl stop nvgetty
sudo systemctl disable nvgetty
sudo udevadm trigger

sudo usermod -a -G tty $USER
sudo usermod -a -G dialout $USER
echo "Reboot to apply changes to serial permissions"
