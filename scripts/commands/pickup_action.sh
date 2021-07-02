TF_NAME=$1  # name of object in the TF tree
OBJECT_CMD="${2:-.NAN}"  # max distance in meters to close the gripper
FORCE_THRESHOLD="${3:-.NAN}"  # max threshold to stop the gripper at (0..1023)
rostopic pub /dodobot/sequence_request/goal db_planning/SequenceRequestActionGoal "{goal: {type: 1, action: 1, name: \"${TF_NAME}\", object_grab_cmd: ${OBJECT_CMD}, force_threshold: ${FORCE_THRESHOLD}}}" -1
