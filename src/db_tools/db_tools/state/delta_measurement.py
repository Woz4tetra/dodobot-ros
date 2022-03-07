from .velocity_filter import VelocityFilter
from .delta_timer import DeltaTimer
from .simplepose3d import SimplePose3d


class DeltaMeasurement:
    def __init__(self, k=None):
        smooth_k = k
        self.vx_filter = VelocityFilter(smooth_k)
        self.vy_filter = VelocityFilter(smooth_k)
        self.vz_filter = VelocityFilter(smooth_k)
        self.timer = DeltaTimer()
        self.state = SimplePose3d()
    
    def set_smooth_k(self, smooth_k):
        self.vx_filter.k = smooth_k
        self.vy_filter.k = smooth_k
        self.vz_filter.k = smooth_k

    def update(self, state: SimplePose3d):
        dt = self.timer.dt(state.stamp)
        if dt == 0.0:
            self.state = state
            return state
        new_state = SimplePose3d.from_state(state)
        new_state.vx = self.vx_filter.update(dt, state.x)
        new_state.vy = self.vy_filter.update(dt, state.y)
        new_state.vz = self.vz_filter.update(dt, state.z)
        self.state = new_state
        return new_state

