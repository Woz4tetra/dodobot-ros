import math
from .robot_controller import RobotController
from .robot_state import Pose2d, Velocity

def sinc(x):
    if math.abs(x) < 1e-9:
        return 1.0 - 1.0 / 6.0 * x * x  # linearized taylor series to avoid singularities
    else:
        return math.sin(x) / x


class RamseteController(RobotController):
    """
    See https://file.tavsys.net/control/controls-engineering-in-frc.pdf
    Controls Engineering in the FIRST Robotics Competition</a> section on 
    Ramsete unicycle controller for a derivation and analysis.
    """
    def __init__(self, b=2.0, zeta=0.7):
        self.b = b
        self.zete = zeta
        super(RamseteController, self).__init__()

    def update(self, current_pose, desired_v):
        if not self.enabled:
            return Velocity()
        if self.arrived():
            return Velocity()

        pose_error = self.goal_pose - self.current_pose

        omega = desired_v.theta
        v_ref = desired_v.x

        k = 2.0 * self.zeta * math.sqrt(omega * omega + self.b * v_ref * v_ref);
        chassis_speeds = Velocity.from_xyt(
            v_ref * math.cos(pose_error.theta) + k * pose_error.x,
            0.0,
            omega + k * pose_error.theta + self.b * self.v_ref * sinc(pose_error.theta) * self.pose_error.y
        )

        return chassis_speeds
