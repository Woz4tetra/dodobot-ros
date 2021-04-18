import math
from .robot_controller import RobotController
from state import Pose2d, Velocity


def sinc(x):
    if abs(x) < 1e-9:
        return 1.0 - 1.0 / 6.0 * x * x  # linearized taylor series to avoid singularities
    else:
        return math.sin(x) / x


class RamseteController(RobotController):
    """
    See https://file.tavsys.net/control/controls-engineering-in-frc.pdf
    Controls Engineering in the FIRST Robotics Competition section on 
    Ramsete unicycle controller for a derivation and analysis.
    """
    def __init__(self, b=2.0, zeta=0.7):
        self.b = b
        self.zeta = zeta
        super(RamseteController, self).__init__()

    def update(self, current_pose, desired_v):
        if not self.enabled:
            return Velocity()
        if self.arrived():
            return Velocity()

        self.current_pose = current_pose

        pose_error = self.goal_pose.relative_to(self.current_pose)
        
        omega = desired_v.theta
        vx = desired_v.x

        k = 2.0 * self.zeta * math.sqrt(omega * omega + self.b * vx * vx)
        chassis_speeds = Velocity.from_xyt(
            vx * math.cos(pose_error.theta) + k * pose_error.x,
            0.0,
            omega + k * pose_error.theta + self.b * vx * sinc(pose_error.theta) * pose_error.y
        )
        chassis_speeds = chassis_speeds.clip(self.lower_v, self.upper_v)

        return chassis_speeds
