NAME=$1

rosservice call /dodobot/db_waypoints/save_robot_pose "{name: $NAME}"
