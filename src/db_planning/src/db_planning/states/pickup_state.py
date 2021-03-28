import rospy
from smach import State


class PickupState(State):
    def __init__(self):
        super(PickupState, self).__init__(
            outcomes=["success", "preempted", "failure"],
            input_keys=["sequence_goal", "central_planning"],
        )
        self.central_planning = None
    
    def execute(self, userdata):
        self.central_planning = userdata.central_planning
        goal = userdata.sequence_goal
        
        # get the goal pose in the linear_link frame
        # linear stepper commands are in the linear_link frame
        goal_pose = self.central_planning.get_goal_pose(goal, self.central_planning.linear_frame)
        if goal_pose is None:
            return "failure"
        
        self.central_planning.set_linear_z(goal_pose.position.z - self.central_planning.pickup_z_offset)
        if not self.central_planning.wait_for_linear_z():
            return "failure"
        
        self.central_planning.close_gripper()
        if not self.central_planning.wait_for_gripper():
            return "failure"
        
        if not self.central_planning.is_gripper_grabbing():
            rospy.logwarn("Gripper failed to sense an object after pickup")
            return "failure"
        
        self.central_planning.set_linear_z(self.central_planning.transport_z_height)
        if not self.central_planning.wait_for_linear_z():
            return "failure"

        return "success"

