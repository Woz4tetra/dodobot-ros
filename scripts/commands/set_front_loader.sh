goal_z=$1
z_speed=${2:-0.04424}
z_accel=${3:-0.0017696}

rostopic pub /dodobot/front_loader_actions/goal db_planning/FrontLoaderActionGoal "{goal: {goal_z: $goal_z, z_speed: $z_speed, z_accel: $z_accel}}" -1
