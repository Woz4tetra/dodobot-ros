import math
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
        self.intermediate_tolerance = rospy.get_param("~intermediate_tolerance", 0.0)
        self.action_result = "success"
        self.goal_pose_stamped = None
        self.current_waypoint_index = 0
        self.num_waypoints = 0

        self.move_base = actionlib.SimpleActionClient(self.move_base_namespace, MoveBaseAction)
        rospy.loginfo("Connecting to move_base...")
        self.move_base.wait_for_server()
        rospy.loginfo("move_base connected")

    def execute(self, userdata):
        self.action_result = "success"
        self.action_server = userdata.action_server

        if userdata.waypoint_index_in >= len(userdata.waypoints):
            self.action_server.set_succeeded(True)
            return "finished"
        
        waypoint_pose = userdata.waypoints[userdata.waypoint_index_in]
        self.goal_pose_stamped = waypoint_pose

        self.num_waypoints = len(userdata.waypoints)
        self.current_waypoint_index = userdata.waypoint_index_in

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = waypoint_pose.header.frame_id
        goal.target_pose.pose.position = waypoint_pose.pose.position
        goal.target_pose.pose.orientation = waypoint_pose.pose.orientation
        
        rospy.loginfo("Going to position (%s, %s)" % (waypoint_pose.pose.position.x, waypoint_pose.pose.position.y))

        self.move_base.send_goal(goal, feedback_cb=self.move_base_feedback, done_cb=self.move_base_done)
        self.move_base.wait_for_result()

        if self.action_result != "success":
            return self.action_result

        move_base_result = self.move_base.get_result()
        # rospy.loginfo("move_base_result: %s" % str(move_base_result))
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
        
        # rospy.loginfo("feedback: %s" % str(feedback))
        if self.intermediate_tolerance != 0.0 and self.current_waypoint_index < self.num_waypoints - 1:
            dist = self.get_xy_dist(feedback.base_position, self.goal_pose_stamped)
            if dist <= self.intermediate_tolerance:
                rospy.loginfo("Robot is close enough to goal. Moving on")
                self.move_base.cancel_goal()
                self.action_result = "success"

    
    def move_base_done(self, goal_status, result):
        rospy.loginfo("move_base finished")
    
    def get_xy_dist(self, pose1, pose2):
        # pose1 and pose2 are PoseStamped
        x1 = pose1.pose.position.x
        y1 = pose1.pose.position.y
        x2 = pose2.pose.position.x
        y2 = pose2.pose.position.y
        
        dx = x2 - x1
        dy = y2 - y1

        return math.sqrt(dx * dx + dy * dy)


class WaypointStateMachine(object):
    def __init__(self):
        self.sm = StateMachine(outcomes=["success", "failure", "preempted"])
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
        rospy.loginfo("To cancel the waypoint follower, run: 'rostopic pub -1 /dodobot/follow_path/cancel actionlib_msgs/GoalID -- {}'")
        rospy.loginfo("To cancel the current goal, run: 'rostopic pub -1 /move_base/cancel actionlib_msgs/GoalID -- {}'")
        self.sm.userdata.sm_waypoint_index = 0
        self.sm.userdata.sm_waypoints = waypoints
        self.sm.userdata.sm_action_server = action_server
        self.outcome = self.sm.execute()
        return self.outcome
