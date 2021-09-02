import rospy
from smach import State
from actionlib_msgs.msg import GoalStatus

from db_planning.msg import SequenceRequestGoal


class PursueDockState(State):
    def __init__(self, central_planning):
        super(PursueDockState, self).__init__(
            outcomes=["success", "too_far", "preempted", "failure"],
            input_keys=["sequence_goal", "central_planning"],
        )
        self.central_planning = central_planning
        self.check_state_interval = 0.1  # seconds
        self.near_object_fudge = 0.05  # fudge factor to avoid dithering between move to object and pursuit
        self.too_close_object = 0.2  # dock is too close for camera to see well. Stop sending goals to pursuit planner
        self.distance_k = 0.9
        self.distance_to_goal = 0.0
    
    def exit_callback(self, goal, outcome):
        self.central_planning.toggle_local_costmap(True)
        self.central_planning.look_straight_ahead()
        if outcome != str(goal.action):
            self.central_planning.set_linear_z_to_transport(goal)
            if not self.central_planning.wait_for_linear_z():
                return "failure"
            return outcome
        else:
            return "success"

    def execute(self, userdata):
        goal = userdata.sequence_goal
        outcome = self.run_pursuit(goal)
        return self.exit_callback(goal, outcome)
    
    def run_pursuit(self, goal):
        # wait for robot to settle and for object filter to update
        rospy.sleep(1.5)
        
        goal_pose = self.central_planning.get_charge_dock_goal()
        if goal_pose is None:
            return "failure"
        rospy.loginfo("Pursuit goal: %0.4f, %0.4f, %0.4f" % (goal_pose.pose.position.x, goal_pose.pose.position.y, goal_pose.pose.position.z))

        self.central_planning.init_pursuit_goal(
            goal_pose,
            backwards_motion_only=True,
            angle_tolerance=0.12,
            position_tolerance=0.07,
            timeout_fudge=1.0,
            timeout_turn_fudge=5.0,
            max_linear_speed=0.15,
            
        )

        while True:
            if rospy.is_shutdown():
                return "preempted"

            rospy.sleep(self.check_state_interval)

            robot_pose = self.central_planning.get_robot_pose()
            distance_to_goal_raw = self.central_planning.get_pose_distance(robot_pose, goal_pose)
            self.distance_to_goal += self.distance_k * (distance_to_goal_raw - self.distance_to_goal)
            rospy.loginfo("Pursuit. Distance to goal: %s" % self.distance_to_goal)

            if self.distance_to_goal > self.too_close_object:
                goal_pose = self.central_planning.get_charge_dock_goal()
                if goal_pose is None:
                    return "failure"
                self.central_planning.set_pursuit_goal(goal_pose)
            
            state = self.central_planning.get_pursuit_state()
            if state == "success":
                return str(goal.action)
            elif state == "failure" or state == "preempted":
                return state

            if self.distance_to_goal > self.central_planning.near_object_distance + self.near_object_fudge:
                rospy.loginfo("Robot has wandered too far away from the goal. Switching from pursuit to move_base")
                self.central_planning.cancel_pursuit_goal()
                return "too_far"
