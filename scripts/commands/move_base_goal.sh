goal_x=$1
goal_y=$2
# goal_angle="${3:-.NAN}"
# drive_forwards="${4:-1}"

rostopic pub /move_base_simple/goal geometry_msgs/PoseStamped "{header: {stamp: now, frame_id: "map"}, pose: {position: {x: $goal_x, y: $goal_y, z: 0.0}, orientation: {w: 1.0}}}"
