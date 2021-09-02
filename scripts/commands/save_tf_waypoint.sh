NAME=$1
FRAME=$2

rosservice call /dodobot/db_waypoints/save_tf "{name: $NAME, frame: $FRAME}"
