#!/usr/bin/env bash

if [ -z $1 ]; then
    LOGS_DIR=${HOME}/logs
    CURRENT_FILE=`find $LOGS_DIR -type f -printf '%T@ %p\n' | sort -n | tail -1 | cut -f2- -d" "`
    echo $CURRENT_FILE
else
    CURRENT_FILE=$1
fi

trap ' ' INT
tail -F -n 500 $CURRENT_FILE
printf "\n\n$CURRENT_FILE\n"
