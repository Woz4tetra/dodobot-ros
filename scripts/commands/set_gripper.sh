distance=$1

rostopic pub /dodobot/gripper_actions/goal db_planning/GripperActionGoal "{goal: {distance: $distance, force_threshold: .NAN}}" -1
