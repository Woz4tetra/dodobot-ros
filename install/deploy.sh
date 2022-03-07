BASE_DIR=$(realpath "$(dirname $0)")
PARENT_DIR=$(dirname $BASE_DIR)
DESTINATION_NAME=$1
REMOTE_KEY=$2
RESTART_ROSLAUNCH=$3

LOCAL_PATH=${PARENT_DIR}
DESTINATION_PATH=$HOME
CATKIN_WS_PATH=$HOME/ros_ws

if [ -z ${DESTINATION_NAME} ]; then
    echo "Please set a destination IP or hostname"
    exit
fi

if [ -z ${REMOTE_KEY} ]; then
    echo "Please set an SSH key file"
    exit
fi

LOCAL_PATH=$(realpath $LOCAL_PATH)
LOCAL_NAME=$(basename $LOCAL_PATH)
DEST_FULL_PATH=${DESTINATION_PATH}/${LOCAL_NAME}

${BASE_DIR}/upload.sh ${DESTINATION_NAME} ${REMOTE_KEY} n

SSH_COMMAND="ssh -i ${REMOTE_KEY} ben@${DESTINATION_NAME}"

# stop roslaunch
echo "Stopping roslaunch"
${SSH_COMMAND} -t "sudo systemctl stop roslaunch.service"

# build db_tools
${SSH_COMMAND} "cd ${DEST_FULL_PATH}/db_tools && python3 setup.py -q install --user"

# build catkin ws
${SSH_COMMAND} -t "cd ${CATKIN_WS_PATH} && source ${CATKIN_WS_PATH}/devel/setup.bash && catkin_make"

${BASE_DIR}/restart.sh ${DESTINATION_NAME} ${REMOTE_KEY} ${RESTART_ROSLAUNCH}
