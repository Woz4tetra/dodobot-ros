
class ChassisState:
    def __init__(self, x=0.0, y=0.0, t=0.0):
        self.x = x
        self.y = y
        self.t = t
        self.vx = 0.0
        self.vt = 0.0

    def __str__(self):
        if self.t is None:
            return "(%s, %s)" % (self.x, self.y)
        else:
            return "(%s, %s, %s)" % (self.x, self.y, self.t)

    def str_vs(self):
        return "(%s, %s)" % (self.vx, self.vt)

    def __add__(self, other):
        return self.__class__(self.x + other.x, self.y + other.y, self.t + other.t)

    def __sub__(self, other):
        return self.__class__(self.x - other.x, self.y - other.y, self.t - other.t)
