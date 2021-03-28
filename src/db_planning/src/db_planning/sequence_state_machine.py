import rospy
from smach import StateMachine
from smach_ros import ActionServerWrapper

from db_planning.states import PrecheckState, MoveToObjectState, PickupState, DeliverState

from db_planning.msg import SequenceRequestAction, SequenceRequestGoal, SequenceRequestResult


class SequenceStateMachine(ActionServerWrapper):
    def __init__(self, central_planning):
        sm = StateMachine(outcomes=["success", "failure", "preempted"])

        with sm:
            StateMachine.add(
                "PRECHECK", PrecheckState(),
                transitions={
                    "success": "MOVE_TO_OBJECT",
                    "failure": "failure",
                },
                remapping={
                    "central_planning": "sm_central_planning",
                    "sequence_goal": "sm_sequence_goal",
                }
            )

            StateMachine.add(
                "MOVE_TO_OBJECT", MoveToObjectState(),
                transitions={
                    SequenceRequestAction.PICKUP: "PICKUP",
                    SequenceRequestAction.DELIVER: "DELIVER",
                    "failure": "failure",
                    "preempted": "preempted"
                },
                remapping={
                    "central_planning": "sm_central_planning",
                    "sequence_goal": "sm_sequence_goal",
                }
            )

            StateMachine.add(
                "PICKUP", PickupState(),
                transitions={
                    "success": "success",
                    "failure": "failure",
                    "preempted": "preempted"
                },
                remapping={
                    "central_planning": "sm_central_planning",
                    "sequence_goal": "sm_sequence_goal",
                }
            )

            StateMachine.add(
                "DELIVER", DeliverState(),
                transitions={
                    "success": "success",
                    "failure": "failure",
                    "preempted": "preempted"
                },
                remapping={
                    "central_planning": "sm_central_planning",
                    "sequence_goal": "sm_sequence_goal",
                }
            )
        
        sm.userdata.sm_central_planning = central_planning
        super(SequenceStateMachine, self).__init__(
            "sequence_request", SequenceRequestAction,
            wrapped_container=sm,
            succeeded_outcomes=["success"],
            aborted_outcomes=["aborted"],
            preempted_outcomes=["preempted"]
        )

    def execute_cb(self, goal):
        rospy.loginfo("To cancel the sequence goal, run: 'rostopic pub -1 /dodobot/sequence_request/cancel actionlib_msgs/GoalID -- {}'")
        self.wrapped_container.userdata.sm_sequence_goal = goal
        super(SequenceStateMachine, self).execute_cb(goal)
