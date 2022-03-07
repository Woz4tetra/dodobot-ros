#!/usr/bin/env bash

echo "Running dodobot firmware upload script"

if systemctl --all --user | grep -Fq 'dodobot_py'; then
    systemctl stop --user dodobot_py.service
fi

platformio run --target upload --upload-port=/dev/serial/by-id/usb-Teensyduino_USB_Serial_6809670-if00

if systemctl --all --user | grep -Fq 'dodobot_py'; then
    systemctl start --user dodobot_py.service
fi
