import math
import rospy
from smach import State
from actionlib_msgs.msg import GoalStatus

from db_planning.msg import SequenceRequestGoal
from db_planning.states.base_pursue_state import BasePursueState


class PursueUnDockState(BasePursueState):
    def __init__(self, central_planning):
        super(PursueUnDockState, self).__init__(
            central_planning,
            ["success"],
            ["preempted", "failure"]
        )
        self.check_state_interval = 0.1  # seconds

    def run_pursuit(self, goal):
        goal_pose = self.central_planning.get_goal_pose(goal)
        if goal_pose is None:
            return "failure"
        goal_pose.pose.orientation = self.central_planning.rotate_quat(goal_pose.pose.orientation, math.pi)
        
        rospy.loginfo("Pursuit goal: %0.4f, %0.4f, %0.4f" % (goal_pose.pose.position.x, goal_pose.pose.position.y, goal_pose.pose.position.z))

        self.central_planning.init_pursuit_goal(
            goal_pose,
            reversed=True,
            angle_tolerance=0.12,
            position_tolerance=0.07,
            timeout_fudge=2.0,
            timeout_turn_fudge=2.0,
            max_linear_speed=0.15,
            loopback_y_tolerance=1.0,
            turn_towards_final_heading=True,
        )

        while True:
            if rospy.is_shutdown():
                return "preempted"

            rospy.sleep(self.check_state_interval)

            state = self.central_planning.get_pursuit_state()
            if state == "success":
                return "success"
            elif state == "failure" or state == "preempted":
                return state
