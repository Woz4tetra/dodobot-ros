import math
import collections
import numpy as np

from state import TrajectoryState


class TimeParameterizer:
    def __init__(self, start_vel=0.0, end_vel=0.0, max_vel=1.0, min_accel=None, max_accel=1.0, is_reversed=False):
        self.points = []
        self.start_vel = start_vel
        self.end_vel = end_vel
        self.max_vel = max_vel
        self.max_accel = max_accel
        self.min_accel = min_accel if min_accel is not None else -max_accel
        self.is_reversed = is_reversed

        assert self.min_accel < 0.0

    def generate(self, points):
        self.points = points
        predessor = TrajectoryState(
            x=self.points[0], y=self.points[1],
            max_vel=self.max_vel, min_accel=self.min_accel, max_accel=self.max_accel
        )

        for point in self.points:
            new_state = TrajectoryState(
                x=point[0], y=point[1],
                max_vel=self.max_vel, min_accel=self.min_accel, max_accel=self.max_accel
            )
            dist = predessor.distance(new_state)
            new_state.traj_dist = dist + predessor.traj_dist
            while True:
                new_state.max_vel = min(self.max_vel, self.project_velocity(predessor.max_vel, predessor.max_accel, dist))
                new_state.min_accel = self.min_accel
                new_state.max_accel = self.max_accel

                # new_state.max_vel = min(self.max_vel, <insert custom velocity constrain>)
                
                if dist < 1E-6:
                    break

                new_state = self.enforce_accel(new_state)
                
                
    def enforce_accel(self, traj_state):
        pass
    
    def project_velocity(self, prev_vel, accel, dist):
        return math.sqrt(prev_vel * prev_vel + 2 * accel * dist)
