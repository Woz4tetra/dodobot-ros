
class State:
    def __init__(self):
        self.x = 0.0  # meters
        self.y = 0.0  # meters
        self.theta = 0.0  # radians
    
    @classmethod
    def from_xyt(cls, x=0.0, y=0.0, theta=0.0):
        self = cls()
        self.x = x
        self.y = y
        self.theta = theta
        return self
    
    @classmethod
    def from_state(cls, state):
        self = cls()
        self.x = state.x
        self.y = state.y
        self.theta = state.theta
        return self

    def __add__(self, other):
        if not isinstance(other, self.__class__):
            raise ValueError("Can't add %s and %s" % (self.__class__, other.__class__))

        self.x += other.x
        self.y += other.y
        self.theta += other.theta

    def __sub__(self, other):
        if not isinstance(other, self.__class__):
            raise ValueError("Can't subtract %s and %s" % (self.__class__, other.__class__))

        self.x -= other.x
        self.y -= other.y
        self.theta -= other.theta
    
    def __mul__(self, other):
        if isinstance(other, self.__class__):
            self.x *= other.x
            self.y *= other.y
            self.theta *= other.theta
        elif isinstance(other, int) or isinstance(other, float):
            self.x *= other
            self.y *= other
            self.theta *= other
        else:
            raise ValueError("Can't multiply %s and %s" % (self.__class__, other.__class__))
    
    def __truediv__(self, other):
        if isinstance(other, self.__class__):
            self.x /= other.x
            self.y /= other.y
            self.theta /= other.theta
        elif isinstance(other, int) or isinstance(other, float):
            self.x /= other
            self.y /= other
            self.theta /= other
        else:
            raise ValueError("Can't divide %s and %s" % (self.__class__, other.__class__))

    
    def __abs__(self):
        return self.__class__.from_xyt(
            abs(self.x),
            abs(self.y),
            abs(self.theta)
        )
    
    def __lt__(self, other):
        return (
            self.x < other.x and
            self.y < other.y and
            self.theta < other.theta
        )
    
    def __eq__(self, other):
        return (
            self.x == other.x and
            self.y == other.y and
            self.theta == other.theta
        )

class Pose2d(State):
    pass

class Velocity(State):
    pass
