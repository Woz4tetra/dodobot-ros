#!/bin/bash
# for serial devices to work (like RPLIDAR), run this command
echo "adding $USER to dialout group"
sudo usermod -a -G dialout $USER
echo "reboot or logout for this to take effect"
