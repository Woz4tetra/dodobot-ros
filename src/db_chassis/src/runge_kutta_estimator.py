import math
from odom_estimator import OdomEstimator


class RungeKuttaEstimator(OdomEstimator):
    def __init__(self, wheel_dist_m):
        super(RungeKuttaEstimator, self).__init__(wheel_dist_m)

    def update(self, **kwargs):
        left_speed = kwargs["left_speed"]
        right_speed = kwargs["right_speed"]
        dt = kwargs["dt"]

        theta_n_1 = self.theta

        v = (left_speed + right_speed) / 2
        w = (right_speed - left_speed) / self.wheel_dist_m

        # reference: https://www.cs.cmu.edu/~16311/current/labs/lab03/
        k00 = v * math.cos(theta_n_1)
        k01 = v * math.sin(theta_n_1)
        k02 = w

        k10 = v * math.cos(theta_n_1 + dt / 2.0 * k02)
        k11 = v * math.sin(theta_n_1 + dt / 2.0 * k02)
        k12 = w

        k20 = v * math.cos(theta_n_1 + dt / 2.0 * k12)
        k21 = v * math.sin(theta_n_1 + dt / 2.0 * k12)
        k22 = w

        k30 = v * math.cos(theta_n_1 + dt / 2.0 * k22)
        k31 = v * math.sin(theta_n_1 + dt / 2.0 * k22)
        k32 = w

        dx     = dt / 6.0 * (k00 + 2 * (k10 + k20) + k30)
        dy     = dt / 6.0 * (k01 + 2 * (k11 + k21) + k31)
        dtheta = dt / 6.0 * (k02 + 2 * (k12 + k22) + k32)

        self.x += dx
        self.y += dy
        self.theta += dtheta
        self.theta %= 2.0 * math.pi

        self.vx = v * math.cos(self.theta)
        self.vy = v * math.sin(self.theta)

        self.v = v
        self.w = w
