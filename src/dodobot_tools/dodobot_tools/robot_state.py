import math
import numpy as np
import tf_conversions
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Pose


class State:
    def __init__(self, x=0.0, y=0.0, theta=0.0):
        self.x = x  # meters
        self.y = y  # meters
        self.theta = theta  # radians

    @classmethod
    def none_pose(cls):
        self = cls()
        self.x = None
        self.y = None
        self.theta = None
        return self

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
    
    @classmethod
    def from_ros_pose(cls, pose):
        self = cls()
        self.x = pose.position.x
        self.y = pose.position.y
        self.theta = State.theta_from_quat(pose.orientation)
        return self
    
    @staticmethod
    def theta_from_quat(quaternion):
        return tf_conversions.transformations.euler_from_quaternion((
            quaternion.x,
            quaternion.y,
            quaternion.z,
            quaternion.w
        ))[2]

    def is_none(self):
        return self.x is None and self.y is None and self.theta is None

    def get_theta_as_quat(self, as_list=False):
        quat = tf_conversions.transformations.quaternion_from_euler(0.0, 0.0, self.theta)
        if as_list:
            return quat
        
        quat_msg = Quaternion()
        quat_msg.x = quat[0]
        quat_msg.y = quat[1]
        quat_msg.z = quat[2]
        quat_msg.w = quat[3]
        return quat_msg


    def relative_to(self, other):
        if not isinstance(other, self.__class__):
            raise ValueError("Can't transform %s to %s" % (self.__class__, other.__class__))
        state = self - other
        state.theta = self.normalize_theta(state.theta)
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

    @staticmethod
    def _clip(x, lower, upper):
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


    def clip(self, lower, upper):
        state = self.__class__.from_state(self)
        if self.magnitude() < lower.magnitude():
            state.theta = State._clip(self.theta, lower.theta, upper.theta)
        if abs(self.theta) < lower.theta:
            state.x = State._clip(self.x, lower.x, upper.x)
            state.y = State._clip(self.y, lower.y, upper.y)
        return state
    
    def magnitude(self):
        return math.sqrt(self.x * self.x + self.y * self.y)
    
    def distance(self, other):
        if not isinstance(other, self.__class__):
            raise ValueError("Can't get distance from %s to %s" % (self.__class__, other.__class__))
        dx = self.x - other.x
        dy = self.y - other.y
        return math.sqrt(dx * dx + dy * dy)
    
    def heading(self, other):
        if not isinstance(other, self.__class__):
            raise ValueError("Can't get heading from %s to %s" % (self.__class__, other.__class__))
        dx = self.x - other.x
        dy = self.y - other.y
        return math.atan2(dy, dx)
    
    @classmethod
    def normalize_theta(cls, theta):
        theta = math.fmod(theta, 2 * math.pi)
        if abs(theta) > math.pi:
            if theta > 0:
                return theta - 2 * math.pi
            else:
                return theta + 2 * math.pi
        return theta
    
    def get_normalize_theta(self):
        return self.normalize_theta(self.theta)
    
    def to_list(self, states="xyt"):
        output = []
        for state in states:
            if state == "x":
                output.append(self.x)
            elif state == "y":
                output.append(self.y)
            elif state == "t":
                output.append(self.theta)
        return output

    @classmethod
    def to_array(cls, poses: list):
        return np.array([pose.to_list() for pose in poses])

    def to_ros_pose(self):
        ros_pose = Pose()
        ros_pose.position.x = self.x
        ros_pose.position.y = self.y
        ros_pose.orientation = self.get_theta_as_quat()
        return ros_pose

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
