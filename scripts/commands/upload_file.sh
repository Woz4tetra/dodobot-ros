#!/usr/bin/env bash
LOCAL_PATH=$1
DEST_NAME=$2
rosservice call /dodobot/dodobot_file "{path: \"$LOCAL_PATH\", dest: \"$DEST_NAME\"}"
