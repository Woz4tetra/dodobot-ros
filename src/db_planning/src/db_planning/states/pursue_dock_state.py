import rospy
from smach import State
from actionlib_msgs.msg import GoalStatus

from db_planning.msg import SequenceRequestGoal
from db_planning.states.base_pursue_state import BasePursueState


class PursueDockState(BasePursueState):
    def __init__(self, central_planning):
        super(PursueDockState, self).__init__(
            central_planning,
            ["success"],
            ["too_far", "preempted", "failure"]
        )
        self.check_state_interval = 0.1  # seconds
        self.near_object_fudge = 0.05  # fudge factor to avoid dithering between move to object and pursuit
        self.too_close_object = 0.2  # dock is too close for camera to see well. Stop sending goals to pursuit planner
        self.distance_k = 0.9
        self.distance_to_goal = 0.0
    
    def ram_dock(self):
        rospy.loginfo("RAMMING DOCK!")
        start_time = rospy.Time.now()
        while not self.central_planning.is_charging:
            self.central_planning.set_twist(0.65, 0.0)
            rospy.sleep(0.1)
            if rospy.Time.now() - start_time > rospy.Duration(10.0):
                break
        self.central_planning.set_twist(0.4, 0.0)
        rospy.sleep(0.75)
        self.central_planning.set_twist(0.0, 0.0)
        
        while self.central_planning.is_robot_moving(time_delay=1.0):
            pass

        if self.central_planning.is_charging:
            rospy.loginfo("Successfully docked with charging station")
            return True
        else:
            rospy.logwarn("Failed docked with charging station!!")
            return False

    def run_pursuit(self, goal):
        # wait for robot to settle and for object filter to update
        rospy.sleep(1.5)
        
        goal_pose = self.central_planning.get_charge_dock_goal()
        rospy.loginfo("Goal pose: %s" % goal_pose)
        if goal_pose is None:
            return "failure"
        rospy.loginfo("Pursuit goal: %0.4f, %0.4f, %0.4f" % (
            goal_pose.pose.position.x,
            goal_pose.pose.position.y,
            self.central_planning.get_euler_z(goal_pose.pose.orientation))
        )

        self.central_planning.init_pursuit_goal(
            goal_pose,
            forwards_motion_only=True,
            angle_tolerance=0.05,
            position_tolerance=0.15,
            timeout_fudge=1.0,
            timeout_turn_fudge=5.0,
            max_linear_speed=0.75,
            max_angular_speed=2.0,
        )

        while True:
            if rospy.is_shutdown():
                return "preempted"

            rospy.sleep(self.check_state_interval)

            robot_pose = self.central_planning.get_robot_pose()
            self.distance_to_goal = self.central_planning.get_pose_distance(robot_pose, goal_pose)
            # self.distance_to_goal += self.distance_k * (distance_to_goal_raw - self.distance_to_goal)
            rospy.loginfo("Pursuit. Distance to goal: %s" % self.distance_to_goal)

            # if self.distance_to_goal > self.too_close_object:
            #     goal_pose = self.central_planning.get_charge_dock_goal()
            #     if goal_pose is None:
            #         return "failure"
            #     self.central_planning.set_pursuit_goal(goal_pose)
            
            state = self.central_planning.get_pursuit_state()
            if state == "success":
                if self.ram_dock():
                    # self.central_planning.set_robot_pose_to_name(self.central_planning.charge_dock_frame)
                    self.central_planning.set_pose_estimate(goal_pose)

                    if self.set_navigation_allowed:
                        if not self.central_planning.set_navigation_idle():
                            return "failure"
                    return "success"
                else:
                    return "failure"
            elif state == "failure" or state == "preempted":
                return state

            if self.distance_to_goal > self.central_planning.near_object_distance + self.near_object_fudge:
                rospy.loginfo("Robot has wandered too far away from the goal. Switching from pursuit to move_base")
                self.central_planning.cancel_pursuit_goal()
                return "too_far"
