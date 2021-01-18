import rospy
import actionlib

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

from smach import State, StateMachine

class GoToWaypointState(State):
    def __init__(self):
        super(GoToWaypointState, self).__init__(
            outcomes=["success", "preempted", "failure", "finished"],
            input_keys=["waypoints", "waypoint_index_in", "action_server"],
            output_keys=["waypoints", "waypoint_index_out", "action_server"]
        )
    
        self.move_base_namespace = rospy.get_param("~move_base_namespace", "/move_base")
        self.goal_tolerance = rospy.get_param("~goal_tolerance", 0.0)
        self.action_result = "success"

        self.move_base = actionlib.SimpleActionClient(self.move_base_namespace, MoveBaseAction)
        rospy.loginfo("Connecting to move_base...")
        self.move_base.wait_for_server()
        rospy.loginfo("move_base connected")


    def execute(self, userdata):
        if userdata.waypoint_index_in >= len(userdata.waypoints):
            return "finished"
        
        waypoint_pose = userdata.waypoints[userdata.waypoint_index_in]

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = waypoint_pose.header.frame_id
        goal.target_pose.pose.position = waypoint_pose.pose.position
        goal.target_pose.pose.orientation = waypoint_pose.pose.orientation
        
        rospy.loginfo("Going to position (%s, %s)" % (waypoint_pose.pose.position.x, waypoint_pose.pose.position.y))
        rospy.loginfo("(Hint) to cancel the goal, run: 'rostopic pub -1 /move_base/cancel actionlib_msgs/GoalID -- {}'")

        self.action_result = "success"
        self.action_server = userdata.action_server

        self.move_base.send_goal(goal, feedback_cb=self.move_base_feedback, done_cb=self.move_base_done)
        self.move_base.wait_for_result()

        if self.action_result != "success":
            return self.action_result

        move_base_result = self.move_action_client.get_result()
        if bool(move_base_result):
            userdata.waypoint_index_out = userdata.waypoint_index_in + 1
            return "success"
        else:
            return "failure"
    
    def move_base_feedback(self, feedback):
        if rospy.is_shutdown():
            rospy.loginfo("Received abort. Cancelling waypoint goal")
            self.action_server.set_aborted()
            self.action_result = "failure"
            self.move_base.cancel_goal()

        if self.action_server.is_preempt_requested():
            rospy.loginfo("Received preempt. Cancelling waypoint goal")
            self.action_server.set_preempted()
            self.action_result = "preempted"
            self.move_base.cancel_goal()
    
    def move_base_done(self):
        rospy.loginfo("move_base finished")


class WaypointStateMachine(object):
    def __init__(self):
        self.sm = StateMachine(outcomes=["success", "failure", "preempted"])
        self.sm.userdata.sm_waypoint_index = 0
        self.outcome = None
        self.action_server = None

        with self.sm:
            StateMachine.add(
                "GOTO_WAYPOINT", GoToWaypointState(),
                transitions={
                    "success": "GOTO_WAYPOINT",
                    "finished": "success",
                    "failure": "failure",
                    "preempted": "preempted"
                },
                remapping={
                    "waypoints": "sm_waypoints",
                    "waypoint_index_in": "sm_waypoint_index",
                    "waypoint_index_out": "sm_waypoint_index",
                    "action_server": "sm_action_server",
                }
            )

    def execute(self, waypoints, action_server):
        self.sm.userdata.sm_waypoints = waypoints
        self.sm.userdata.sm_action_server = action_server
        self.outcome = self.sm.execute()
        return self.outcome
