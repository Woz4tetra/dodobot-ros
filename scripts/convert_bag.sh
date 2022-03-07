#!/usr/bin/env bash

BAGS_DIR=${HOME}/bags
CONVERT_BAG=$1
if [ -z ${CONVERT_BAG} ]; then
    LATEST_BAG=`find $BAGS_DIR -type f -name *.bag -printf '%T@ %p\n' | sort -n | tail -1 | cut -f2- -d" "`
    echo $LATEST_BAG
    CONVERT_BAG=LATEST_BAG
else
    CONVERT_BAG=$BAGS_DIR/$CONVERT_BAG
fi
cd ~/dodobot-ros/src/db_tools/db_tools/rosbag_to_file
python3 convert.py -a -o /media/storage/bags --path $CONVERT_BAG
