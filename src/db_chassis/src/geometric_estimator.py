import math
from odom_estimator import OdomEstimator


class GeometricEstimator(OdomEstimator):
    def __init__(self, wheel_dist_m):
        super(GeometricEstimator, self).__init__(wheel_dist_m)

    def update(self, **kwargs):
        delta_left = kwargs["delta_left"]
        delta_right = kwargs["delta_right"]

        left_speed = kwargs["left_speed"]
        right_speed = kwargs["right_speed"]

        delta_dist = (delta_right + delta_left) / 2

        # angle = arc / radius
        delta_angle = (delta_right - delta_left) / self.wheel_dist_m
        self.theta += delta_angle
        self.theta %= 2.0 * math.pi

        dx = delta_dist * math.cos(self.theta)
        dy = delta_dist * math.sin(self.theta)

        self.x += dx
        self.y += dy

        self.v = (left_speed + right_speed) / 2
        self.vx = self.v * math.cos(self.theta)
        self.vy = self.v * math.sin(self.theta)
        self.w = (right_speed - left_speed) / self.wheel_dist_m
