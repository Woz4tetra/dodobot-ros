from __future__ import division, print_function
import math
from db_planning.chassis_state import ChassisState
import rospy

class TwistController(object):
    def __init__(self):
        self.max_linear_speed = 1E9
        self.min_linear_speed = 0.0
        self.max_angular_speed = 1E9
        self.min_angular_speed = 0.0
        self.max_linear_acceleration = 1E9
        self.max_angular_acceleration = 1E9
        self.linear_tolerance = 0.025  # 2.5cm
        self.angular_tolerance = math.radians(3.0)  # 3 degrees
        self.forward_movement_only = False
        self.reverse_direction = False

        self.dist_timeout_tolerance = 0.0005
        self.angle_timeout_tolerance = 0.0025
        self.prev_dist_error = 0.0
        self.prev_angle_error = 0.0
        self.dist_error = 0.0
        self.angle_error = 0.0
        self.dist_timeout_timer = None
        self.angle_timeout_timer = None
        self.move_timeout = rospy.Duration(10.0)
    
    def get_command(self, current_state, goal_state, dT):
        return ChassisState()
    
    def is_stuck(self):
        now = rospy.Time.now()
        if self.dist_timeout_timer is None:
            self.dist_timeout_timer = now
        if abs(self.dist_error - self.prev_dist_error) < self.dist_timeout_tolerance:
            if now - self.dist_timeout_timer > self.move_timeout:
                return True
        else:
            self.dist_timeout_timer = now

        # if self.angle_timeout_timer is None:
        #     self.angle_timeout_timer = now
        # if abs(self.angle_error - self.prev_angle_error) < self.angle_timeout_tolerance:
        #     if now - self.angle_timeout_timer > self.move_timeout:
        #         return True
        # else:
        #     self.angle_timeout_timer = now

        self.prev_dist_error = self.dist_error
        # self.prev_angle_error = self.angle_error


    def at_goal(self, cur, goal):
        if goal is None:
            return True

        self.dist_error = self.get_goal_distance(cur, goal)
        if goal.t is None:
            is_at_goal = self.dist_error < self.linear_tolerance
        else:
            self.angle_error = abs(self.normalize_pi(cur.t - goal.t))
            is_at_goal = self.dist_error < self.linear_tolerance and self.angle_error < self.angular_tolerance

        if is_at_goal:
            self.dist_timeout_timer = None
        return is_at_goal

    def get_goal_distance(self, cur, goal):
        if goal is None:
            return 0
        diffX = cur.x - goal.x
        diffY = cur.y - goal.y
        return math.sqrt(diffX * diffX + diffY * diffY)

    def normalize_half_pi(self, alpha):
        alpha = self.normalize_pi(alpha)
        if alpha > math.pi / 2:
            return alpha - math.pi
        elif alpha < -math.pi / 2:
            return alpha + math.pi
        else:
            return alpha

    def normalize_pi(self, alpha):
        alpha %= 2.0 * math.pi

        if alpha >= math.pi:
            alpha -= 2.0 * math.pi

        return alpha
