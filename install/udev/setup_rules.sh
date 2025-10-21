#!/bin/bash
# Setup Teensy USB rules
wget -q https://www.pjrc.com/teensy/00-teensy.rules -O 49-teensy.rules
sudo mv 49-teensy.rules /etc/udev/rules.d/49-teensy.rules
sudo usermod -aG dialout $USER

# Setup UART rules
sudo tee -a /etc/udev/rules.d/99-jetson-uart.rules > /dev/null << 'EOF'
KERNEL=="ttyTHS[0-9]*", MODE="0666"
EOF

# Reload rules
sudo udevadm control --reload-rules
sudo udevadm trigger
