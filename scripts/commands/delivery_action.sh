TF_NAME=$1
rostopic pub /dodobot/sequence_request/goal db_planning/SequenceRequestActionGoal "{goal: {sequence_type: \"delivery\", sequence_goal: \"${TF_NAME}\"}}" -1
