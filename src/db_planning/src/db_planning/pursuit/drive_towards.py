import math
import rospy
from .pursuit_action import PursuitAction
from db_planning.robot_state import Pose2d


class DriveTowards(PursuitAction):
    def __init__(self, parameters, set_velocity, get_state, should_stop):
        super(DriveTowards, self).__init__(parameters, set_velocity, get_state, should_stop)

    def get_error(self, state: Pose2d) -> Pose2d:
        error = self.goal_pose.relative_to(state)
        target_angle = self.goal_pose.heading(state)
        if self.parameters.reversed:
            target_angle += math.pi
        error.theta = Pose2d.normalize_theta(target_angle - state.theta)
        if self.parameters.forwards_motion_only and error.x < 0.0:
            error.x = 0.0
        
        rospy.loginfo("%s error: %s, state: %s" % (self.__class__.__name__, error, state))

        return error

    def get_timeout(self, error):
        time_to_goal = abs(error.x) / self.parameters.min_linear_speed
        time_to_goal *= self.parameters.timeout_safety_factor
        time_to_goal += self.parameters.stabilization_timeout
        time_to_goal += self.parameters.timeout_fudge
        return time_to_goal

    def update(self, state: Pose2d, error: Pose2d):
        # if the robot's y or theta errors exceed certain values, jump back to turning in place
        # if (abs(error.y) > self.parameters.loopback_y_tolerance or
        #         abs(error.theta) > self.parameters.loopback_theta_tolerance):
        if abs(error.y) > self.parameters.loopback_y_tolerance:
            rospy.loginfo("Robot deviated too far from the straight path. Turning in place.")
            self.stop_motors()
            rospy.sleep(0.5)  # wait for object filter lag to catch up
            return "turn"

        return None

    def is_goal_reached(self, error: Pose2d):
        return abs(error.x) < self.parameters.position_tolerance

    def get_command(self, error):
        err_t_ang_v = self.parameters.fine_steer_kP * error.theta
        err_y_ang_v = self.parameters.steer_kP * error.y
        ang_v = err_t_ang_v + err_y_ang_v
        linear_v = self.parameters.linear_kP * error.x
        if self.parameters.forwards_motion_only:
            if self.parameters.reversed and linear_v > 0.0:
                linear_v = 0.0
            elif linear_v < 0.0:
                linear_v = 0.0
        
        if self.parameters.reversed:
            ang_v *= -1.0
        return linear_v, ang_v
