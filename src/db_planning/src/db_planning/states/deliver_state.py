import rospy
from smach import State


class DeliverState(State):
    def __init__(self, central_planning):
        super(DeliverState, self).__init__(
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
        
        self.central_planning.set_linear_z(
            goal_pose.pose.position.z + self.central_planning.deliver_z_offset + goal.object_z_offset
        )
        if not self.central_planning.wait_for_linear_z():
            return "failure"
        
        self.central_planning.open_gripper()
        if not self.central_planning.wait_for_gripper():
            return "failure"
        
        if self.central_planning.is_gripper_grabbing():
            rospy.logwarn("Gripper failed to release an object after delivery")
            return "failure"
        
        self.central_planning.set_linear_z(self.central_planning.transport_z_height, self.central_planning.fast_stepper_speed)
        if not self.central_planning.wait_for_linear_z():
            return "failure"

        return "success"

