TF_NAME=$1
rostopic pub /dodobot/sequence_request/goal db_planning/SequenceRequestActionGoal "{goal: {type: 1, action: 1, name: \"${TF_NAME}\"}}" -1
