TF_NAME=$1
OBJECT_Z="${2:-0}"
rostopic pub /dodobot/sequence_request/goal db_planning/SequenceRequestActionGoal "{goal: {type: 1, action: 2, name: \"${TF_NAME}\", object_z_offset: ${OBJECT_Z}}}" -1
