
class OdomEstimator(object):
    def __init__(self, wheel_dist_m):
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.v = 0.0
        self.w = 0.0

        self.vx = 0.0
        self.vy = 0.0

        self.wheel_dist_m = wheel_dist_m

    def update(self, **kwargs):
        raise NotImplementedError

    def reset(self, x, y, theta):
        self.x = x
        self.y = y
        self.theta = theta

        self.v = 0.0
        self.w = 0.0

        self.vx = 0.0
        self.vy = 0.0

    def __str__(self):
        return "x: %0.4f\ty: %0.4f\tt: %0.4f" % (self.x, self.y, self.theta)

    def __repr__(self):
        return "%s(x=%0.4f,\ty=%0.4f,\tt=%0.4f,\tvx=%0.4f,\tvy=%0.4f,\tv=%0.4f,\tw=%0.4f)" % (
            self.__class__.__name__,
            self.x, self.y, self.theta,
            self.vx, self.vy, self.v, self.w
        )
