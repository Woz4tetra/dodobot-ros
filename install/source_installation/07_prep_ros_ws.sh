#!/usr/bin/env bash


SOURCE_COMMAND='source ${HOME}/ros_ws/devel/setup.bash
source ${HOME}/packages_ros_ws/devel/setup.bash
source ${HOME}/tj2_ros/systemd/env.sh'

if ! grep -qz "$SOURCE_COMMAND" ~/.bashrc; then
    echo "Appending ros setup scripts to ~/.bashrc"
    echo "$SOURCE_COMMAND" | sudo tee -a ~/.bashrc > /dev/null
fi

WS_DIR=${HOME}/ros_ws/src
mkdir -p ${WS_DIR}
ln -s ${HOME}/tj2_ros ${WS_DIR}

echo "re-open this terminal session for bashrc to go into affect"
