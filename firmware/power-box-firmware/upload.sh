#!/usr/bin/env bash

echo "Running power box firmware upload script"

platformio run --target upload --upload-port=/dev/serial/by-id/usb-Silicon_Labs_CP2104_USB_to_UART_Bridge_Controller_018D432A-if00-port0
