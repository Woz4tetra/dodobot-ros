# imported from https://github.com/merose/diff_drive

from __future__ import division, print_function
import math
from db_planning.chassis_state import ChassisState
from .twist_controller import TwistController
import rospy

class SiegwartController(TwistController):
    """Finds linear and angular velocities necessary to drive toward
    a goal pose.
    """

    def __init__(self):
        self.kP = 3.0
        self.kA = 8.0
        self.kB = -1.5
        super(SiegwartController, self).__init__()

    def set_constants(self, kP, kA, kB):
        self.kP = kP
        self.kA = kA
        self.kB = kB

    def get_command(self, cur, goal, dT):
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
