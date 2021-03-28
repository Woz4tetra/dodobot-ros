import rospy
from smach import State

from db_planning.msg import SequenceRequestAction


class PrecheckState(State):
    def __init__(self):
        super(PrecheckState, self).__init__(
            outcomes=["success", "failure"],
            input_keys=["sequence_goal", "central_planning"],
        )
        self.central_planning = None

    def execute(self, userdata):
        self.central_planning = userdata.central_planning
        if self.sequence_prechecks(userdata.sequence_goal):
            return "success"
        else:
            return "failure"

    def sequence_prechecks(self, goal):
        result = self.central_planning.get_robot_state()
        if not result.active:
            rospy.logwarn("Motors are not active! Cannot start action.")
            return False

        result = self.central_planning.front_loader_ready_srv()
        if not result.success:
            rospy.logwarn("Front loader is not ready! Cannot start action: %s" % result.message)
            return False
        
        if goal.action not in self.central_planning.valid_action_types:
            rospy.logwarn("Invalid action type: %s" % goal.action)
            return False
        
        if goal.goal_type not in self.central_planning.valid_goal_types:
            rospy.logwarn("Invalid goal type: %s" % goal.goal_type)
            return False
        
        if not self.central_planning.is_gripper_ok(goal):
            rospy.logwarn("Gripper is not ready to start sequence")
            return False
        
        if not self.central_planning.does_goal_exist(goal):
            rospy.logwarn("Goal does not exist: %s"  % (
                goal.pose_stamped if goal.goal_type == SequenceRequestAction.POSE_GOAL else goal.name
            ))
            return False

        return True
