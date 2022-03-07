BASE_DIR=$(realpath "$(dirname $0)")

cp ${BASE_DIR}/ros_joystick.yaml ~/.local/dodobot/dodobot_py/config/joystick.yaml

if systemctl --all --user | grep -Fq 'dodobot_py'; then
    systemctl restart --user dodobot_py.service
fi

