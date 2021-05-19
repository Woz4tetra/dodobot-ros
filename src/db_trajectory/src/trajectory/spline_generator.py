import numpy as np
from state import Pose2d, Velocity
from scipy.interpolate import CubicHermiteSpline, CubicSpline


class SplineGenerator:
    def __init__(self, waypoints, spline_time=1.0, spline_magnitude=1.2, use_full_pose=True):
        self.waypoints = waypoints  # List[Pose2d]

        self.spline_start_time = 0.0
        self.spline_stop_time = spline_time
        self.spline_times = np.linspace(self.spline_start_time, self.spline_stop_time, len(self.waypoints))
        self.xs = np.array([[waypoint.x, waypoint.y] for waypoint in self.waypoints])
        if use_full_pose:
            self.dxdt = np.array([
                [spline_magnitude * np.cos(waypoint.theta),
                spline_magnitude * np.sin(waypoint.theta)] for waypoint in self.waypoints]
            )
            self.spline = CubicHermiteSpline(self.spline_times, self.xs, self.dxdt, axis=0)
        else:
            self.dxdt = None
            self.spline = CubicSpline(self.spline_times, self.xs, axis=1)
    
    def __getitem__(self, spline_time):
        return self.spline(spline_time)
    
    def iter(self, num_points):
        spline_times = np.linspace(self.spline_start_time, self.spline_stop_time, num_points)
        for spline_time in spline_times:
            yield self.spline(spline_time)
