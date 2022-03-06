#!/usr/bin/env bash

BASE_DIR=$(realpath "$(dirname $0)")

DEPENDENCIES_WS=$HOME/packages_ros_ws
DEPENDENCIES_WS_SRC=${DEPENDENCIES_WS}/src


for dir in ${DEPENDENCIES_WS_SRC}/* ; do
    cd $dir
    echo $dir
    git pull
done

cd ${DEPENDENCIES_WS}

rosdep install --from-paths src --ignore-src --rosdistro=noetic -y -r

catkin_make -j5
success=$?
if [[ $success -eq 0 ]];
then
    echo "Packages built successfully"
else
    echo "Something went wrong!"
    exit 1
fi
