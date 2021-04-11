TF_NAME=$1
OBJECT_CMD="${2:-.NAN}"
FORCE_THRESHOLD="${3:-.NAN}"
rostopic pub /dodobot/sequence_request/goal db_planning/SequenceRequestActionGoal "{goal: {type: 1, action: 1, name: \"${TF_NAME}\", object_grab_cmd: ${OBJECT_CMD}, force_threshold: ${FORCE_THRESHOLD}}}" -1
