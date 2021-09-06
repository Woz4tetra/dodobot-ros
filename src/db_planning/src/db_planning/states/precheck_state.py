import rospy
from smach import State

from db_planning.msg import SequenceRequestGoal


class PrecheckState(State):
    def __init__(self, central_planning):
        super(PrecheckState, self).__init__(
            outcomes=["success_park", "success_dock", "success_undock", "success_object", "failure"],
            input_keys=["sequence_goal"],
        )
        self.central_planning = central_planning

    def execute(self, userdata):
        goal = userdata.sequence_goal

        if not self.central_planning.is_navigation_running():
            if self.set_navigation_allowed:
                if not self.set_navigation_active():
                    return "failure"
            else:
                rospy.logwarn("Navigation is not active! Cannot start action.")
                return "failure"

        if not self.central_planning.are_drive_motors_active():
            if self.central_planning.set_motor_state_allowed:
                self.central_planning.set_drive_motors_active(True)
            else:
                rospy.logwarn("Motors are not active! Cannot start action.")
                return "failure"

        if goal.action != SequenceRequestGoal.UNDOCK:
            result = self.central_planning.get_linear_stepper_ready_state()
            if not result.success:
                if self.central_planning.home_linear_allowed:
                    self.central_planning.home_linear_stepper()
                else:
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
        
        if goal.action != SequenceRequestGoal.UNDOCK:
            self.central_planning.set_linear_z_to_transport(goal)
            if not self.central_planning.wait_for_linear_z():
                return "failure"
            
        if not self.central_planning.does_goal_exist(goal):
            rospy.logwarn("Goal does not exist: %s"  % (
                goal.pose_stamped if goal.type == SequenceRequestGoal.POSE_GOAL else goal.name
            ))
            return "failure"

        if goal.action == SequenceRequestGoal.PARK:
            return "success_park"
        elif goal.action == SequenceRequestGoal.DOCK:
            return "success_dock"
        elif goal.action == SequenceRequestGoal.UNDOCK:
            return "success_undock"
        else:
            return "success_object"
