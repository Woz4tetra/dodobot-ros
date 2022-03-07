import rospy

from smach import State


class CheckWaypointState(State):
    """
    Checks the current waypoint to see if it's a spot on the map or the name of an object to pursue
    """
    def __init__(self, state_machine):
        self.state_machine = state_machine
        super(CheckWaypointState, self).__init__(
            outcomes=["waypoint", "object", "finished", "failure", "preempted"],
            input_keys=["waypoints_plan", "waypoint_index"],
            output_keys=["waypoints_plan", "waypoint_index"]
        )

    def execute(self, userdata):
        rospy.loginfo("Checking waypoint")

        waypoints_node = self.state_machine.waypoints_node

        self.num_waypoints = len(userdata.waypoints_plan)
        current_waypoint_index = userdata.waypoint_index

        if current_waypoint_index >= self.num_waypoints:
            return "finished"

        waypoints = userdata.waypoints_plan[userdata.waypoint_index]

        first_waypoint = waypoints[0]
        if waypoints_node.is_waypoint(first_waypoint.name):
            return "waypoint"
        elif waypoints_node.is_object(first_waypoint.name):
            return "object"
        else:
            rospy.logerr("'%s' is not an object or registered waypoint" % first_waypoint.name)
            return "failure"
