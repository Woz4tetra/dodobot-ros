# imported from https://github.com/merose/diff_drive

from __future__ import division, print_function
import math
from db_planning.chassis_state import ChassisState
import rospy

class GoalController:
    """Finds linear and angular velocities necessary to drive toward
    a goal pose.
    """

    def __init__(self):
        self.kP = 3.0
        self.kA = 8.0
        self.kB = -1.5

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
        self.dist_timeout_timer = rospy.Time.now()
        self.angle_timeout_timer = rospy.Time.now()
        self.move_timeout = rospy.Duration(10.0)

    def set_constants(self, kP, kA, kB):
        self.kP = kP
        self.kA = kA
        self.kB = kB

    def get_goal_distance(self, cur, goal):
        if goal is None:
            return 0
        diffX = cur.x - goal.x
        diffY = cur.y - goal.y
        return math.sqrt(diffX * diffX + diffY * diffY)

    def at_goal(self, cur, goal):
        if goal is None:
            return True

        self.dist_error = self.get_goal_distance(cur, goal)
        if goal.t is None:
            return self.dist_error < self.linear_tolerance
        else:
            self.angle_error = abs(self.normalize_pi(cur.t - goal.t))
            return self.dist_error < self.linear_tolerance and self.angle_error < self.angular_tolerance

    def is_stuck(self):
        now = rospy.Time.now()
        if abs(self.dist_error - self.prev_dist_error) < self.dist_timeout_tolerance:
            if now - self.dist_timeout_timer > self.move_timeout:
                return True
        else:
            self.dist_timeout_timer = now

        # if abs(self.angle_error - self.prev_angle_error) < self.angle_timeout_tolerance:
        #     if now - self.angle_timeout_timer > self.move_timeout:
        #         return True
        # else:
        #     self.angle_timeout_timer = now

        self.prev_dist_error = self.dist_error
        # self.prev_angle_error = self.angle_error

    def get_velocity(self, cur, goal, dT):
        """
        cur: ChassisState
        goal: ChassisState
        dT: float
        """
        desired = ChassisState()

        goal_heading = math.atan2(goal.y - cur.y, goal.x - cur.x)
        if self.reverse_direction:
            goal_heading = self.normalize_pi(goal_heading + math.pi)
        a = -cur.t + goal_heading

        if goal.t is None:
            goal_t = goal_heading
        else:
            goal_t = goal.t

        # In Automomous Mobile Robots, they assume theta_G=0. So for
        # the error in heading, we have to adjust theta based on the
        # (possibly non-zero) goal theta.
        theta = self.normalize_pi(cur.t - goal_t)
        b = -theta - a

        # rospy.loginfo('cur=%f goal=%f a=%f b=%f', cur.theta, goal_heading,
        #               a, b)

        d = self.get_goal_distance(cur, goal)
        if self.forward_movement_only:
            direction = 1.0
            a = self.normalize_pi(a)
            b = self.normalize_pi(b)
        else:
            direction = math.copysign(1.0, math.cos(a))
            a = self.normalize_half_pi(a)
            b = self.normalize_half_pi(b)

        if self.reverse_direction:
            direction *= -1.0

        # rospy.loginfo('After normalization, a=%f b=%f', a, b)

        if abs(d) < self.linear_tolerance:
            desired.vx = 0.0
            desired.vt = self.kB * theta
        else:
            desired.vx = self.kP * d * direction
            desired.vt = self.kA * a + self.kB * b

        # TBD: Adjust velocities if linear or angular acceleration
        # too high.

        # Adjust velocities if X velocity is too high.
        if abs(desired.vx) > self.max_linear_speed:
            ratio = self.max_linear_speed / abs(desired.vx)
            desired.vx *= ratio
            desired.vt *= ratio

        # Adjust velocities if turning velocity too high.
        if abs(desired.vt) > self.max_angular_speed:
            ratio = self.max_angular_speed / abs(desired.vt)
            desired.vx *= ratio
            desired.vt *= ratio

        # Adjust velocities if too low, so robot does not stall.
        if abs(desired.vx) > 0 and abs(desired.vx) < self.min_linear_speed:
            ratio = self.min_linear_speed / abs(desired.vx)
            desired.vx *= ratio
            desired.vt *= ratio
        elif desired.vx == 0.0 and desired.vt != 0.0 and abs(desired.vt) < self.min_angular_speed:
            ratio = self.min_angular_speed / abs(desired.vt)
            desired.vx *= ratio
            desired.vt *= ratio

        return desired

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
