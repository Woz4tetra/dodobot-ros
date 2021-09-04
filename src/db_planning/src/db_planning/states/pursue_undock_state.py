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
        # self.central_planning.set_twist(-0.75, 0.0)
        # rospy.sleep(1.0)
        # self.central_planning.set_twist(0.0, 0.0)
        # rospy.sleep(1.0)

        result = self.central_planning.drive_straight(-0.3)
        if result == "success":
            result = self.central_planning.turn_in_place(math.pi)
        return result
