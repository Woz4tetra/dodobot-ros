import rospy
from smach import State
from actionlib_msgs.msg import GoalStatus

from db_planning.msg import SequenceRequestGoal


class PursueObjectState(State):
    def __init__(self, central_planning):
        super(PursueObjectState, self).__init__(
            outcomes=[str(SequenceRequestGoal.PICKUP), str(SequenceRequestGoal.DELIVER), "too_far", "preempted", "failure"],
            input_keys=["sequence_goal", "central_planning"],
        )
        self.central_planning = central_planning
        self.check_state_interval = 0.25  # seconds
        self.near_object_fudge = 0.05  # fudge factor to avoid dithering between move to object and pursuit
    
    def exit_callback(self, goal, outcome):
        self.central_planning.toggle_local_costmap(True)
        self.central_planning.look_straight_ahead()
        if outcome != str(goal.action):
            self.central_planning.set_linear_z_to_transport(goal)
            if not self.central_planning.wait_for_linear_z():
                return "failure"
        return outcome

    def execute(self, userdata):
        goal = userdata.sequence_goal
        outcome = self.run_pursuit(goal)
        return self.exit_callback(goal, outcome)
    
    def run_pursuit(self, goal):
        goal_pose = self.central_planning.get_nav_goal(goal)
        # goal_pose = self.central_planning.get_goal_pose_with_gripper(goal)
        if goal_pose is None:
            return "failure"
        rospy.loginfo("Pursuit goal: %0.4f, %0.4f, %0.4f" % (goal_pose.pose.position.x, goal_pose.pose.position.y, goal_pose.pose.position.z))

        goal_orientation = goal_pose.pose.orientation

        self.central_planning.toggle_local_costmap(False)
        if goal.action == SequenceRequestGoal.PICKUP:
            self.central_planning.set_linear_z_to_object_height(goal_pose, goal, self.central_planning.fast_stepper_speed)

        self.central_planning.init_pursuit_goal(goal_pose)

        while True:
            if rospy.is_shutdown():
                return "preempted"

            rospy.sleep(self.check_state_interval)

            robot_pose = self.central_planning.get_robot_pose()
            distance_to_goal = self.central_planning.get_pose_distance(robot_pose, goal_pose)
            rospy.loginfo("Pursuit. Distance to goal: %s" % distance_to_goal)
            self.central_planning.look_at_goal(goal)  # tilt the camera towards the goal
            goal_pose = self.central_planning.get_goal_with_orientation(goal, goal_orientation)
            self.central_planning.set_pursuit_goal(goal_pose)
            
            state = self.central_planning.get_pursuit_state()
            if state == "success":
                return str(goal.action)
            elif state == "failure" or state == "preempted":
                return state

            if distance_to_goal > self.central_planning.near_object_distance + self.near_object_fudge:
                rospy.loginfo("Robot has wandered too far away from the goal. Switching from pursuit to move_base")
                self.central_planning.cancel_pursuit_goal()
                return "too_far"
