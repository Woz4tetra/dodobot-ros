# Based on this paper: https://www.dis.uniroma1.it/~labrob/pub/papers/Ramsete01.pdf

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
        self.zeta = 0.3
        self.b = 25.5
        super(SiegwartController, self).__init__()

    def set_constants(self, zeta, b):
        self.zeta = zeta
        self.b = b

    @staticmethod
    def gain_fn(vx, vt, b, zeta):
        return 2.0 * zeta * math.sqrt(vt * vt + b * vx * vx)

    def get_command(self, cur, goal, dT):
        """
        cur: ChassisState
        goal: ChassisState
        dT: float
        """
        desired = ChassisState()

        t_err = cur.t - goal.t
        x_err = cur.x - goal.y
        y_err = cur.y - goal.y
        # From section 5.12, https://www.dis.uniroma1.it/~labrob/pub/papers/Ramsete01.pdf
        desired.vx = (goal.vx * math.cos(t_err) + 
            self.gain_fn(goal.vx, goal.vt, self.b, self.zeta) * 
            (x_err * math.cos(goal.t) + y_err * math.sin(goal.t))
        )
        desired.vt = (goal.vt + 
            (self.b * goal.vx * math.sin(t_err) / t_err) * 
            (y_err * math.cos(goal.t) - x_err * math.sin(goal.t)) + 
            self.gain_fn(goal.vx, goal.vt, self.b, self.zeta) * t_err
        )

        return desired