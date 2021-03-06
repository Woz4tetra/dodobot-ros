import rospy
from .pursuit_action import PursuitAction
from db_planning.robot_state import Pose2d


class TurnTowards(PursuitAction):
    def __init__(self, parameters, set_velocity, get_state, should_stop):
        super(TurnTowards, self).__init__(parameters, set_velocity, get_state, should_stop)

    def get_error(self, state: Pose2d) -> Pose2d:
        target_angle = self.goal_pose.heading(state)
        error_angle = Pose2d.normalize_theta(target_angle - state.theta)
        error = Pose2d.from_state(state)
        error.theta = error_angle
        return error

    def get_timeout(self, error):
        time_to_goal = abs(error.theta) / self.parameters.min_angular_speed
        time_to_goal *= self.parameters.timeout_safety_factor
        time_to_goal += self.parameters.stabilization_timeout
        time_to_goal += self.parameters.timeout_turn_fudge
        return time_to_goal

    def update(self, state: Pose2d, error: Pose2d):
        return None

    def is_goal_reached(self, error: Pose2d):
        return abs(error.theta) < self.parameters.angle_tolerance

    def get_command(self, error: Pose2d):
        ang_v = self.parameters.steer_kP * error.theta
        return 0.0, ang_v
