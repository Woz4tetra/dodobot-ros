
class VelocityFilter:
    def __init__(self, k):
        self.k = k
        self.prev_value = None
        self.speed = 0.0

    def update(self, dt, value):
        if self.prev_value is None:
            self.prev_value = value
        raw_delta = (value - self.prev_value) / dt
        self.prev_value = value
        if self.k is None or self.k == 0.0:
            self.speed = raw_delta
        else:
            self.speed = self.k * (raw_delta - self.speed)
        return self.speed
