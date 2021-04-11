import rospy
from smach import State
from actionlib_msgs.msg import GoalStatus

from db_planning.msg import SequenceRequestGoal


class MoveToObjectState(State):
    def __init__(self, central_planning):
        super(MoveToObjectState, self).__init__(
            outcomes=[str(SequenceRequestGoal.PICKUP), str(SequenceRequestGoal.DELIVER), "preempted", "failure"],
            input_keys=["sequence_goal", "central_planning"],
        )
        self.central_planning = central_planning
        self.check_state_interval = 0.25  # seconds
        self.should_replan = True
    
    def execute(self, userdata):
        goal = userdata.sequence_goal
        if goal.type == SequenceRequestGoal.NAMED_GOAL:
            pass
        elif goal.type == SequenceRequestGoal.POSE_GOAL:
            raise NotImplementedError
        
        self.central_planning.set_planner_velocities(
            max_vel_x=self.central_planning.max_vel_x,
            max_vel_theta=self.central_planning.max_vel_theta,
        )
        
        goal_pose = self.central_planning.get_move_base_goal(goal)
        if goal_pose is None:
            return "failure"
        rospy.loginfo("Start goal: %0.4f, %0.4f, %0.4f" % (goal_pose.pose.position.x, goal_pose.pose.position.y, goal_pose.pose.position.z))

        # if goal.action == SequenceRequestGoal.PICKUP:
        #     self.central_planning.set_linear_z_to_object_height(goal_pose, goal, self.central_planning.fast_stepper_speed)
        # elif goal.action == SequenceRequestGoal.DELIVER:
        #     self.central_planning.set_linear_z(self.central_planning.transport_z_height, self.central_planning.slow_stepper_speed)
            
        # if not self.central_planning.wait_for_linear_z():
        #     return "failure"

        goal_orientation = goal_pose.pose.orientation
        self.central_planning.set_move_base_goal(goal_pose)
        
        while True:
            if rospy.is_shutdown():
                return "preempted"
            
            state = self.central_planning.get_move_base_state()
            if state == GoalStatus.SUCCEEDED:
                self.central_planning.look_straight_ahead()
                return str(goal.action)
            elif state == GoalStatus.ABORTED:
                self.central_planning.look_straight_ahead()
                return "preempted"
            # elif state == GoalStatus.ACTIVE:

            rospy.sleep(self.check_state_interval)

            # If the robot is near the object, start updating move_base's goal position.
            # Point camera at where the object might be.
            # This avoids the robot getting distracted by false positives along the path
            robot_pose = self.central_planning.get_robot_pose()
            distance_to_goal = self.central_planning.get_pose_distance(robot_pose, goal_pose)
            rospy.loginfo("distance_to_goal: %s" % distance_to_goal)
            if distance_to_goal < self.central_planning.local_costmap_width / 2.0:
                self.central_planning.toggle_local_costmap(False)
                self.central_planning.look_at_goal(goal)  # tilt the camera towards the goal
            else:
                self.central_planning.toggle_local_costmap(True)
                self.central_planning.look_straight_ahead()

            if distance_to_goal < self.central_planning.near_object_distance:
                # Use the original orientation to compute the goal.
                # This avoids situations where the make_plan distance offset is greater than the robot's distance
                # to the goal. In this case, this final orientation wouldn't be valid
                # if distance_to_goal < self.central_planning.replan_distance and self.should_replan:
                if self.should_replan:
                    self.central_planning.cancel_move_base()
                    rospy.sleep(0.5)
                    if goal.action == SequenceRequestGoal.PICKUP:
                        self.central_planning.set_linear_z_to_object_height(goal_pose, goal, self.central_planning.fast_stepper_speed)

                    goal_pose = self.central_planning.get_goal_with_orientation(goal, goal_orientation)
                    rospy.loginfo("Replanning based on new goal: %0.4f, %0.4f" % (goal_pose.pose.position.x, goal_pose.pose.position.y))
                    if goal_pose is None:
                        return "failure"
                    self.central_planning.set_move_base_goal(goal_pose)
                    self.should_replan = False
            else:
                self.should_replan = True
