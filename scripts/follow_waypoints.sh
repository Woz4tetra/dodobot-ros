rostopic pub /dodobot/follow_path/goal db_waypoints/FollowPathActionGoal "{goal: {waypoints: {waypoints: [{name: $1}]}, intermediate_tolerance: ${3:-0.25}}}" -1
