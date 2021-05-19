from .robot_state import State


class TrajectoryState(State):
    def __init__(self, x=0.0, y=0.0, theta=0.0, curvature=0.0, dist=0.0, max_vel=0.0, min_accel=0.0, max_accel=0.0):
        self.curvature = curvature
        self.dist = dist
        self.max_vel = max_vel
        self.min_accel = min_accel
        self.max_accel = max_accel

        super(TrajectoryState, self).__init__(x, y, theta)
