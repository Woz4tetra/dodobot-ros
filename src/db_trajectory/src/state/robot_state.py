import math


def clip(x, lower, upper):
    if x == 0.0:
        return x
    if lower is None and upper is None:
        return x
    elif lower is None and upper is not None:
        clipped_x = min(abs(x), upper)
    if lower is not None and upper is None:
        clipped_x = max(abs(x), lower)
    else:
        clipped_x = min(max(abs(x), lower), upper)
    clipped_x = math.copysign(clipped_x, x)
    return clipped_x


def wrap_angle(angle):
    angle = math.fmod(angle, 2 * math.pi)

    if abs(angle) > math.pi:
        if angle > 0:
            return angle - 2 * math.pi
        else:
            return angle + 2 * math.pi

    return angle


class State:
    def __init__(self, x=0.0, y=0.0, theta=0.0):
        self.x = x  # meters
        self.y = y  # meters
        self.theta = theta  # radians
    
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
    
    def relative_to(self, other):
        if not isinstance(other, self.__class__):
            raise ValueError("Can't transform %s to %s" % (self.__class__, other.__class__))
        state = self - other
        state.theta = wrap_angle(state.theta)
        return state.rotate_by(-other.theta)
    
    def rotate_by(self, theta):
        """
        Apply rotation matrix (defined by theta)
        """
        state = self.__class__()
        state.x = self.x * math.cos(theta) - self.y * math.sin(theta)
        state.y = self.x * math.sin(theta) + self.y * math.cos(theta)
        state.theta = self.theta
        return state

    def clip(self, lower, upper):
        state = self.__class__.from_state(self)
        if self.magnitude() < lower.magnitude():
            state.theta = clip(self.theta, lower.theta, upper.theta)
        if abs(self.theta) < lower.theta:
            state.x = clip(self.x, lower.x, upper.x)
            state.y = clip(self.y, lower.y, upper.y)
        return state
    
    def magnitude(self):
        return math.sqrt(self.x * self.x + self.y * self.y)
    
    def distance(self, other):
        if not isinstance(other, self.__class__):
            raise ValueError("Can't get distance from %s to %s" % (self.__class__, other.__class__))
        dx = self.x - other.x
        dy = self.y - other.y
        return math.sqrt(dx * dx + dy * dy)

    def __add__(self, other):
        if not isinstance(other, self.__class__):
            raise ValueError("Can't add %s and %s" % (self.__class__, other.__class__))

        state = self.__class__()
        state.x = self.x + other.x
        state.y = self.y + other.y
        state.theta = self.theta + other.theta
        return state

    def __sub__(self, other):
        if not isinstance(other, self.__class__):
            raise ValueError("Can't subtract %s and %s" % (self.__class__, other.__class__))

        state = self.__class__()
        state.x = self.x - other.x
        state.y = self.y - other.y
        state.theta = self.theta - other.theta
        return state
    
    def __mul__(self, other):
        state = self.__class__()
        if isinstance(other, self.__class__):
            state.x = self.x * other.x
            state.y = self.y * other.y
            state.theta = self.theta * other.theta
        elif isinstance(other, int) or isinstance(other, float):
            state.x = self.x * other
            state.y = self.y * other
            state.theta = self.theta * other
        else:
            raise ValueError("Can't multiply %s and %s" % (self.__class__, other.__class__))
        return state
    
    def __truediv__(self, other):
        state = self.__class__()
        if isinstance(other, self.__class__):
            state.x = self.x / other.x
            state.y = self.y / other.y
            state.theta = self.theta / other.theta
        elif isinstance(other, int) or isinstance(other, float):
            state.x = self.x / other
            state.y = self.y / other
            state.theta = self.theta / other
        else:
            raise ValueError("Can't divide %s and %s" % (self.__class__, other.__class__))
        return state

    
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
    
    def __str__(self):
        return "%s(x=%0.4f, y=%0.4f, theta=%0.4f)" % (self.__class__.__name__, self.x, self.y, self.theta)
    
    __repr__ = __str__

class Pose2d(State):
    pass

class Velocity(State):
    pass
