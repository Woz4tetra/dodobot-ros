import math
import rospy


class PID(object):
    def __init__(self, Kp, Kd, Ki, K_ff=0.0):
        self.upper_bound = None
        self.lower_bound = None

        self.error_sum = 0.0
        self.prev_error = 0.0
        self.out = 0.0
        self.min_out = 0.0
        self.deadzone = 0.0

        self.Kp = Kp
        self.Kd = Kd
        self.Ki = Ki

    def set_bounds(self, lower, upper):
        self.upper_bound = upper
        self.lower_bound = lower

    def set_deadzones(self, deadzone, min_out):
        assert deadzone < min_out
        self.min_out = min_out
        self.deadzone = deadzone

    def limit(self, value):
        if self.upper_bound is not None and value > self.upper_bound:
            return self.upper_bound
        elif self.lower_bound is not None and value < self.lower_bound:
            return self.lower_bound
        return value

    def reset(self):
        self.error_sum = 0.0
        self.prev_error = 0.0
        self.out = 0.0

    def update(self, error, dt):
        self.out = 0.0

        if (self.Kp != 0.0):
            self.out += self.Kp * error
        if (self.Kd != 0.0):
            self.out += self.Kd * (error - self.prev_error) / dt
            self.prev_error = error
        if (self.Ki != 0.0):
            self.out += self.Ki * self.error_sum * dt
            self.error_sum += error

        if abs(self.out) < self.deadzone:
            self.out = 0.0

        if abs(self.out) < self.min_out:
            self.out = math.copysign(self.min_out, self.out)

        return self.limit(self.out)
