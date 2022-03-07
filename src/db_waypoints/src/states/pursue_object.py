import rospy

from smach import State

from db_pursuit.msg import PursueObjectGoal


class PursueObjectState(State):
    """
    Runs the pursuit planner searching for detections with the supplied waypoint name
    """
    def __init__(self, state_machine):
        self.state_machine = state_machine
        super(PursueObjectState, self).__init__(
            outcomes=["success", "preempted", "failure"],
            input_keys=["waypoints_plan", "waypoint_index"],
            output_keys=["waypoints_plan", "waypoint_index"]
        )
        self.search_timeout = rospy.Duration(60.0)
        self.goal_tolerance = 0.1
        self.pursuit_action = None
        self.action_server = None
        self.is_pursuit_done = False

    def execute(self, userdata):
        rospy.loginfo("Going to object")
        self.pursuit_action = self.state_machine.waypoints_node.pursuit_action
        self.is_pursuit_done = False
        self.action_server = self.state_machine.waypoints_node.follow_path_server

        waypoints = userdata.waypoints_plan[userdata.waypoint_index]
        first_waypoint = waypoints[0]  # objects don't have a continuous mode for now. Only use the first waypoint
        
        goal = PursueObjectGoal()
        goal.object_name = first_waypoint.name
        rospy.loginfo("Pursuit object name: %s" % goal.object_name)
        goal.timeout = self.search_timeout
        goal.xy_tolerance = self.goal_tolerance
        self.pursuit_action.send_goal(goal, done_cb=self.pursuit_done)
        wait_result = self.wait_for_pursuit()
        if wait_result != "success":
            return wait_result
        result = self.pursuit_action.get_result()
        if result.success:
            rospy.loginfo("Object successfully reached")
        else:
            rospy.logwarn("Failed to reach object")
        return "success"

    def pursuit_done(self, goal_status, result):
        rospy.loginfo("pursuit finished: %s. %s" % (goal_status, result))
        self.is_pursuit_done = True

    def wait_for_pursuit(self):
        rate = rospy.Rate(10.0)
        while not self.is_pursuit_done:
            rate.sleep()
            if rospy.is_shutdown():
                rospy.loginfo("Received abort. Cancelling pursuit goal")
                self.pursuit_action.cancel_goal()
                return "failure"

            if self.action_server.is_preempt_requested():
                rospy.loginfo("Received preempt. Cancelling pursuit goal")
                self.pursuit_action.cancel_goal()
                return "preempted"
        return "success"

