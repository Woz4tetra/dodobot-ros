import math
import rospy
from smach import State


class PickupState(State):
    def __init__(self, central_planning):
        super(PickupState, self).__init__(
            outcomes=["success", "preempted", "failure"],
            input_keys=["sequence_goal", "central_planning"],
        )
        self.central_planning = central_planning
    
    def execute(self, userdata):
        goal = userdata.sequence_goal
        
        # get the goal pose in the linear_link frame
        # linear stepper commands are in the linear_link frame
        goal_pose = self.central_planning.get_goal_pose(goal, self.central_planning.linear_frame)
        if goal_pose is None:
            return "failure"
        
        self.central_planning.set_linear_z_to_object_height(goal_pose, goal, self.central_planning.fast_stepper_speed)
        if not self.central_planning.wait_for_linear_z():
            return "failure"
        
        object_grab_cmd = 0.0 if math.isnan(goal.object_grab_cmd) else goal.object_grab_cmd
        force_threshold = None if math.isnan(goal.force_threshold) else goal.force_threshold
        self.central_planning.close_gripper(object_grab_cmd, force_threshold)
        if not self.central_planning.wait_for_gripper():
            return "failure"
        
        self.central_planning.set_linear_z(self.central_planning.transport_z_height, self.central_planning.slow_stepper_speed)
        if not self.central_planning.wait_for_linear_z():
            return "failure"

        if not self.central_planning.is_gripper_grabbing():
            rospy.logwarn("Gripper failed to sense an object after pickup")
            return "success"
            # return "failure"  # force sensor's aren't reliable
        
        return "success"

