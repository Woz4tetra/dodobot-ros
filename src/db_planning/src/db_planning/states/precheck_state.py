import rospy
from smach import State

from db_planning.msg import SequenceRequestGoal


class PrecheckState(State):
    def __init__(self, central_planning):
        super(PrecheckState, self).__init__(
            outcomes=["success", "failure"],
            input_keys=["sequence_goal"],
        )
        self.central_planning = central_planning

    def execute(self, userdata):
        goal = userdata.sequence_goal
        result = self.central_planning.get_robot_state()
        if not result.active:
            rospy.logwarn("Motors are not active! Cannot start action.")
            return "failure"

        result = self.central_planning.front_loader_ready_srv()
        if not result.success:
            rospy.logwarn("Front loader is not ready! Cannot start action: %s" % result.message)
            return "failure"
        
        if goal.action not in self.central_planning.valid_action_types:
            rospy.logwarn("Invalid action type: %s" % goal.action)
            return "failure"
        
        if goal.type not in self.central_planning.valid_goal_types:
            rospy.logwarn("Invalid goal type: %s" % goal.type)
            return "failure"
        
        if not self.central_planning.is_gripper_ok(goal):
            rospy.logwarn("Gripper is not ready to start sequence")
            # return "failure"  # FSRs are unreliable. Ignoring this condition
        
        if goal.action == SequenceRequestGoal.PICKUP:
            self.central_planning.open_gripper()
            if not self.central_planning.wait_for_gripper():
                return "failure"
        
        if goal.action == SequenceRequestGoal.PICKUP:
            stepper_speed = self.central_planning.fast_stepper_speed
        elif goal.action == SequenceRequestGoal.DELIVER:
            stepper_speed = self.central_planning.slow_stepper_speed
        else:
            stepper_speed = float("nan")
        self.central_planning.set_linear_z(self.central_planning.transport_z_height, stepper_speed)
        if not self.central_planning.wait_for_linear_z():
            return "failure"
        
        if not self.central_planning.does_goal_exist(goal):
            rospy.logwarn("Goal does not exist: %s"  % (
                goal.pose_stamped if goal.type == SequenceRequestGoal.POSE_GOAL else goal.name
            ))
            return "failure"

        return "success"
