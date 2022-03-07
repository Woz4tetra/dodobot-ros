
class DeltaTimer:
    def __init__(self):
        self.prev_stamp = None

    def dt(self, timestamp):
        if self.prev_stamp is None:
            self.prev_stamp = timestamp
        dt = timestamp - self.prev_stamp
        self.prev_stamp = timestamp
        return dt

