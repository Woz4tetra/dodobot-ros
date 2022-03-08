#!/bin/bash
# for serial devices to work (like RPLIDAR), run this command
BASE_DIR=$(realpath "$(dirname $0)")

echo "adding $USER to dialout group"
sudo usermod -a -G dialout $USER

OUTPUT_FILE=/etc/rc.local

sudo touch $OUTPUT_FILE

while IFS="" read -r p || [ -n "$p" ]
do
    printf '%s\n' "$p"

    if grep -Fq "$p" $OUTPUT_FILE ; then
        echo "command already exists in $OUTPUT_FILE"
    else
        printf '%s\n' "$p" | sudo tee -a $OUTPUT_FILE > /dev/null
    fi
done < ${BASE_DIR}/rclocal_append.txt


echo "reboot for this to take effect"
