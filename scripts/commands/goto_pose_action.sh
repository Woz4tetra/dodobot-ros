goal_x=$1
goal_y=$2
goal_angle="${3:-.NAN}"
drive_forwards="${4:-1}"
pos_tolerance=0.01
angle_tolerance=0.02
echo ${drive_forwards}

# rosservice call /dodobot/tag_conversion_active "{data: true}"

rostopic pub /dodobot/chassis_actions/goal db_planning/ChassisActionGoal "{goal: {goal_x: $goal_x, goal_y: $goal_y, goal_angle: $goal_angle, base_speed: 0.15, base_ang_v: 0.7, pos_tolerance: $pos_tolerance, angle_tolerance: $angle_tolerance, drive_forwards: $drive_forwards}}" -1
