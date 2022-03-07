import math
import time
import rospy

from geometry_msgs.msg import PoseStamped

from move_base_msgs.msg import MoveBaseGoal

from smach import State


class GoToWaypointState(State):
    def __init__(self, state_machine):
        self.state_machine = state_machine
        super(GoToWaypointState, self).__init__(
            outcomes=["success", "preempted", "failure", "object"],
            input_keys=["waypoints_plan", "waypoint_index"],
            output_keys=["waypoints_plan", "waypoint_index"]
        )
        self.reset()
    
    def reset(self):
        self.action_result = "success"
        self.goal_pose_stamped = None
        self.current_waypoint_index = 0
        self.num_waypoints = 0

        self.action_server = None
        self.move_base = None

        self.intermediate_tolerance = 0.0
        self.ignore_orientation = False
        self.ignore_obstacles = False
        self.interruptable_by = ""

        self.intermediate_settle_time = rospy.Duration(0.25)  # TODO: change to launch param or goal param
        self.within_range_time = None
        
        self.is_move_base_done = False
        self.distance_to_goal = None
        self.epsilon = 1E-100

        self.costmap_toggle_delay = 0.5
    
    def set_ignore_obstacles(self, userdata, ignore_obstacles):
        self.state_machine.waypoints_node.toggle_obstacles(not ignore_obstacles)

    def set_ignore_layers(self, userdata, ignore_walls, ignore_obstacles):
        did_change = self.state_machine.waypoints_node.toggle_walls(not ignore_walls)
        if ignore_walls:
            return self.set_ignore_obstacles(userdata, True) or did_change
        return self.set_ignore_obstacles(userdata, ignore_obstacles)

    def execute(self, userdata):
        rospy.loginfo("Going to waypoint")
        self.reset()

        self.action_result = "success"
        waypoints_node = self.state_machine.waypoints_node
        self.action_server = waypoints_node.follow_path_server
        self.action_goal = self.state_machine.action_goal
        self.move_base = waypoints_node.move_base

        self.current_waypoint_index = userdata.waypoint_index

        waypoints = userdata.waypoints_plan[userdata.waypoint_index]

        pose_array = waypoints_node.get_waypoints_as_pose_array(waypoints)
        if len(pose_array.poses) != len(waypoints):
            rospy.logwarn("Some waypoints are ignored!")
        if len(pose_array.poses) == 0:
            return "success"

        first_waypoint = waypoints[0]  # for some parameters, only the first waypoint's values matter

        self.is_move_base_done = False
        self.distance_to_goal = None
        self.within_range_time = None

        self.intermediate_tolerance = first_waypoint.intermediate_tolerance
        self.ignore_orientation = first_waypoint.ignore_orientation
        self.ignore_obstacles = first_waypoint.ignore_obstacles
        self.ignore_walls = first_waypoint.ignore_walls
        self.interruptable_by = first_waypoint.interruptable_by

        if self.set_ignore_layers(userdata, self.ignore_walls, self.ignore_obstacles):
            time.sleep(self.costmap_toggle_delay)

        if self.ignore_orientation and abs(self.intermediate_tolerance) < self.epsilon:
            self.intermediate_tolerance = 0.075  # TODO: pull this from move_base local planner parameters
        
        self.goal_pose_stamped = PoseStamped()
        self.goal_pose_stamped.header = pose_array.header
        self.goal_pose_stamped.pose = pose_array.poses[-1]

        # forked version of move_base: https://github.com/frc-88/navigation
        # In this version, move_base accepts pose arrays. If continuous mode is enabled,
        # waypoints are all used together in the global plan instead of discrete move_base
        # action calls
        goal = MoveBaseGoal()
        goal.target_poses.header.frame_id = pose_array.header.frame_id
        goal.target_poses = pose_array
        
        rospy.loginfo("Going to position (%s, %s)" % (self.goal_pose_stamped.pose.position.x, self.goal_pose_stamped.pose.position.y))

        self.move_base.send_goal(goal, feedback_cb=self.move_base_feedback, done_cb=self.move_base_done)
        wait_result = self.wait_for_move_base()
        if wait_result == "object":
            rospy.loginfo("Found %s closer than waypoint goal. Pursuing that instead" % self.interruptable_by)
            waypoints[0].name = self.interruptable_by  # set the current waypoint to the object name instead for PursueObjectState
            return "object"

        if self.action_result != "success":
            return self.action_result

        move_base_result = self.move_base.get_result()
        if bool(move_base_result):
            return "success"
        else:
            rospy.loginfo("move_base result was not a success")
            return "failure"

    def wait_for_move_base(self):
        rate = rospy.Rate(10.0)
        while not self.is_move_base_done:
            if rospy.is_shutdown():
                rospy.loginfo("Received abort. Cancelling waypoint goal")
                self.action_result = "failure"
                self.move_base.cancel_goal()
                break

            if self.action_server.is_preempt_requested():
                rospy.loginfo("Received preempt. Cancelling waypoint goal")
                self.action_result = "preempted"
                self.move_base.cancel_goal()
                break

            # if ignore_orientation is True
            #   if this is the last waypoint
            #       wait for the robot to settle, then cancel goal
            #   if this isn't the last waypoint
            #       cancel goal
            # if ignore_orientation is False
            #   if intermediate_tolerance is greater than zero
            #       cancel goal
            if self.distance_to_goal is None:
                continue
                
            nearest_obj_dist = self.get_nearest_object_distance(self.interruptable_by)
            if nearest_obj_dist is not None:
                if nearest_obj_dist < self.distance_to_goal:
                    return "object"

            if self.distance_to_goal <= self.intermediate_tolerance:
                if self.within_range_time is None:
                    self.within_range_time = rospy.Time.now()

                if self.ignore_orientation:
                    if self.current_waypoint_index < self.num_waypoints - 1:
                        if rospy.Time.now() - self.within_range_time > self.intermediate_settle_time:
                            self.close_enough_to_goal()
                    else:
                        self.close_enough_to_goal()
                else:
                    if self.intermediate_tolerance > 0.0:
                        self.close_enough_to_goal()
            else:
                self.within_range_time = None
            
            rate.sleep()
        return "success"
        
    def move_base_feedback(self, feedback):
        self.distance_to_goal = self.get_xy_dist(feedback.base_position, self.goal_pose_stamped)
    
    def get_nearest_object_distance(self, object_name):
        if len(object_name) == 0:
            return None
        return 10000.0  # TODO: fill with detection results

    def close_enough_to_goal(self):
        rospy.loginfo("Robot is close enough to goal: %s. Moving on" % self.distance_to_goal)
        self.move_base.cancel_goal()
        self.action_result = "success"
    
    def move_base_done(self, goal_status, result):
        rospy.loginfo("move_base finished: %s. %s" % (goal_status, result))
        self.is_move_base_done = True
    
    def get_xy_dist(self, pose1, pose2):
        # pose1 and pose2 are PoseStamped
        x1 = pose1.pose.position.x
        y1 = pose1.pose.position.y
        x2 = pose2.pose.position.x
        y2 = pose2.pose.position.y
        
        dx = x2 - x1
        dy = y2 - y1

        return math.sqrt(dx * dx + dy * dy)
