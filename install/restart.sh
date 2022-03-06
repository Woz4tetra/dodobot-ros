DESTINATION_NAME=$1
REMOTE_KEY=$2
RESTART_ROSLAUNCH=$3

if [ -z ${DESTINATION_NAME} ]; then
    echo "Please set a destination IP or hostname"
    exit
fi

if [ -z ${REMOTE_KEY} ]; then
    echo "Please set an SSH key file"
    exit
fi

SSH_COMMAND="ssh -i ${REMOTE_KEY} -p 5810 tj2@${DESTINATION_NAME}"

# restart systemd
if [ -z $RESTART_ROSLAUNCH ]; then
    echo "Restart roslaunch.service? (Y/n) "
    read response
    case $response in
      ([Nn])     echo "Skipping restart";;
      (*)        echo "Restarting roslaunch." && ${SSH_COMMAND} -t "sudo systemctl restart roslaunch.service";;
    esac
else
    if [[ $RESTART_ROSLAUNCH == "n" ]]; then
        echo "Skipping restart"
    else
        echo "Restarting roslaunch." && ${SSH_COMMAND} -t "sudo systemctl restart roslaunch.service"
    fi
fi
