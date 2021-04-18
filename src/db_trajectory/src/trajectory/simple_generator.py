from state import Pose2d, Velocity
from scipy.interpolate import CubicSpline


class SimpleGenerator:
    def __init__(self, start: Pose2d, end: Pose2d):
        self.start = start
        self.end = end

        self.CubicSpline(x, y)
