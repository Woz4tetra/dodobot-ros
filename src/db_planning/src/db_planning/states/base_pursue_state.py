import rospy
from smach import State
from actionlib_msgs.msg import GoalStatus

from db_planning.msg import SequenceRequestGoal


class BasePursueState(State):
    def __init__(self, central_planning, success_outcomes, failure_outcomes):
        self.success_outcomes = success_outcomes
        self.failure_outcomes = failure_outcomes
        outcomes = list(success_outcomes) + list(failure_outcomes)
        super(BasePursueState, self).__init__(
            outcomes=outcomes,
            input_keys=["sequence_goal", "central_planning"],
        )
        self.central_planning = central_planning

    def exit_callback(self, goal, success):
        if self.central_planning.is_move_base_running():
            self.central_planning.toggle_local_costmap(True)
        self.central_planning.look_straight_ahead()
        if not success:
            self.central_planning.set_linear_z_to_transport(goal)
            if not self.central_planning.wait_for_linear_z():
                return False
        return True

    def execute(self, userdata):
        goal = userdata.sequence_goal
        outcome = self.run_pursuit(goal)
        exit_outcome = self.exit_callback(goal, outcome in self.success_outcomes)
        if exit_outcome:
            return outcome
        else:
            return "failure"
    
    def run_pursuit(self, goal):
        return "success"
