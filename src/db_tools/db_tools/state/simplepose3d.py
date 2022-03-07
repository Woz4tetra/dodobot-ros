import time
import math
import numpy as np
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from vision_msgs.msg import Detection2D
from vision_msgs.msg import Detection3D
from .pose2d import Pose2d
from scipy.spatial.transform import Rotation


class SimplePose3d(Pose2d):
    def __init__(self, x=0.0, y=0.0, z=0.0, theta=0.0, vx=0.0, vy=0.0, vz=0.0, vt=0.0):
        super(SimplePose3d, self).__init__(x, y, theta)
        self.type = ""
        self.stamp = 0.0
        self.z = z
        self.vx = vx
        self.vy = vy
        self.vz = vz
        self.vt = vt

    @classmethod
    def from_state(cls, other):
        if not isinstance(other, cls):
            raise ValueError("%s is not of type %s" % (repr(other), cls))
        self = cls()
        self.type = other.type
        self.stamp = other.stamp
        self.x = other.x
        self.y = other.y
        self.z = other.z
        self.theta = other.theta
        self.vx = other.vx
        self.vy = other.vy
        self.vz = other.vz
        self.vt = other.vt
        return self

    @classmethod
    def from_ros_pose(cls, pose):
        self = cls()
        self.x = pose.position.x
        self.y = pose.position.y
        self.z = pose.position.z
        self.theta = cls.theta_from_quat(pose.orientation)
        return self

    @classmethod
    def from_odom(cls, msg):
        if not isinstance(msg, Odometry):
            raise ValueError("%s is not of type %s" % (repr(msg), Odometry))

        self = cls.from_ros_pose(msg.pose.pose)
        self.type = "odom"
        self.stamp = msg.header.stamp.to_sec()
        self.vx = msg.twist.twist.linear.x
        self.vy = msg.twist.twist.linear.y
        self.vz = msg.twist.twist.linear.z
        self.vt = msg.twist.twist.angular.z
        return self

    @classmethod
    def from_detect(cls, msg):
        if not (isinstance(msg, Detection2D) or isinstance(msg, Detection3D)):
            raise ValueError("%s is not of type %s" % (repr(msg), Detection2D))
        self = cls.from_ros_pose(msg.results[0].pose.pose)
        self.type = msg.results[0].id
        self.stamp = msg.header.stamp.to_sec()
        return self

    def distance(self, other=None):
        if other is None:  # if other is None, assume you're getting distance from the origin
            other = self.__class__()
        if not isinstance(other, self.__class__):
            raise ValueError("Can't get distance from %s to %s" % (self.__class__, other.__class__))
        dx = self.x - other.x
        dy = self.y - other.y
        dz = self.z - other.z
        return math.sqrt(dx * dx + dy * dy + dz * dz)

    def velocity_magnitude(self, other=None):
        if other is None:  # if other is None, assume you're getting distance from the origin
            other = self.__class__()
        if not isinstance(other, self.__class__):
            raise ValueError("Can't get distance from %s to %s" % (self.__class__, other.__class__))
        dvx = self.vx - other.vx
        dvy = self.vy - other.vy
        dvz = self.vz - other.vz
        return math.sqrt(dvx * dvx + dvy * dvy + dvz * dvz)

    def to_ros_pose(self):
        ros_pose = Pose()
        ros_pose.position.x = self.x
        ros_pose.position.y = self.y
        ros_pose.position.z = self.z
        ros_pose.orientation = self.get_theta_as_quat()
        return ros_pose

    def relative_to(self, other):
        if not isinstance(other, self.__class__):
            raise ValueError("%s is not of type %s" % (repr(other), self.__class__))
        new_self = self.__class__.from_state(other)
        new_self.stamp = self.stamp

        rot_mat = Rotation.from_euler("z", other.theta).as_matrix()
        point = np.array([self.x, self.y, self.z])
        tf_point = np.dot(rot_mat, point)
        new_self.x = tf_point[0] + other.x
        new_self.y = tf_point[1] + other.y
        new_self.z = tf_point[2] + other.z
        
        velocity = np.array([self.vx, self.vy, self.vz])
        tf_vel = np.dot(rot_mat, velocity)
        new_self.vx = tf_vel[0] + other.vx
        new_self.vy = tf_vel[1] + other.vy
        new_self.vz = tf_vel[2] + other.vz

        new_self.theta = self.theta + other.theta
        new_self.vt = self.vt + other.vt

        return new_self

    def relative_to_reverse(self, other):
        if not isinstance(other, self.__class__):
            raise ValueError("%s is not of type %s" % (repr(other), self.__class__))
        new_self = self.__class__.from_state(other)
        new_self.stamp = self.stamp

        other_reverse = -other

        rot_mat = Rotation.from_euler("z", other_reverse.theta).as_matrix()
        
        point = np.array([
            self.x + other_reverse.x,
            self.y + other_reverse.y,
            self.z + other_reverse.z
        ])
        tf_point = np.dot(rot_mat, point)
        new_self.x = tf_point[0]
        new_self.y = tf_point[1]
        new_self.z = tf_point[2]

        velocity = np.array([
            self.vx + other_reverse.vx,
            self.vy + other_reverse.vy,
            self.vz + other_reverse.vz
        ])
        tf_vel = np.dot(rot_mat, velocity)
        new_self.vx = tf_vel[0]
        new_self.vy = tf_vel[1]
        new_self.vz = tf_vel[2]

        new_self.theta = self.theta + other_reverse.theta
        new_self.vt = self.vt + other_reverse.vt

        return new_self

    def __str__(self):
        return f"{self.__class__.__name__}<{self.type}>(" \
               f"x={self.x:0.4f}, y={self.y:0.4f}, z={self.z:0.4f}, t={self.theta:0.4f}, " \
               f"vx={self.vx:0.4f}, vy={self.vy:0.4f}, vz={self.vz:0.4f}, vt={self.vt:0.4f}) @ {self.stamp}"

    __repr__ = __str__

    def __pos__(self):
        return self

    def __neg__(self):
        new_state = self.from_state(self)
        new_state.x = -new_state.x
        new_state.y = -new_state.y
        new_state.z = -new_state.z
        new_state.theta = -new_state.theta
        new_state.vx = -new_state.vx
        new_state.vy = -new_state.vy
        new_state.vz = -new_state.vz
        new_state.vt = -new_state.vt
        return new_state

    def __add__(self, other):
        if not isinstance(other, self.__class__):
            raise ValueError("Can't add %s and %s" % (self.__class__, other.__class__))

        state = self.__class__()
        state.x = self.x + other.x
        state.y = self.y + other.y
        state.z = self.y + other.z
        state.theta = self.theta + other.theta
        state.vx = self.vx + other.vx
        state.vy = self.vy + other.vy
        state.vz = self.vz + other.vz
        state.vt = self.vt + other.vt
        return state

    def __sub__(self, other):
        if not isinstance(other, self.__class__):
            raise ValueError("Can't subtract %s and %s" % (self.__class__, other.__class__))

        state = self.__class__()
        state.x = self.x - other.x
        state.y = self.y - other.y
        state.z = self.z - other.z
        state.theta = self.theta - other.theta
        state.vx = self.vx - other.vx
        state.vy = self.vy - other.vy
        state.vz = self.vz - other.vz
        state.vt = self.vt - other.vt
        return state

    def __mul__(self, other):
        state = self.__class__()
        if isinstance(other, self.__class__):
            state.x = self.x * other.x
            state.y = self.y * other.y
            state.z = self.z * other.z
            state.theta = self.theta * other.theta
            state.vx = self.vx * other.vx
            state.vy = self.vy * other.vy
            state.vz = self.vz * other.vz
            state.vt = self.vt * other.vt
        elif isinstance(other, int) or isinstance(other, float):
            state.x = self.x * other
            state.y = self.y * other
            state.z = self.z * other
            state.theta = self.theta * other
            state.vx = self.vx * other
            state.vy = self.vy * other
            state.vz = self.vz * other
            state.vt = self.vt * other
        else:
            raise ValueError("Can't multiply %s and %s" % (self.__class__, other.__class__))
        return state

    def __truediv__(self, other):
        state = self.__class__()
        if isinstance(other, self.__class__):
            state.x = self.x / other.x
            state.y = self.y / other.y
            state.z = self.z / other.z
            state.theta = self.theta / other.theta
            state.vx = self.vx / other.vx
            state.vy = self.vy / other.vy
            state.vz = self.vz / other.vz
            state.vt = self.vt / other.vt
        elif isinstance(other, int) or isinstance(other, float):
            state.x = self.x / other
            state.y = self.y / other
            state.z = self.z / other
            state.theta = self.theta / other
            state.vx = self.vx / other
            state.vy = self.vy / other
            state.vz = self.vz / other
            state.vt = self.vt / other
        else:
            raise ValueError("Can't divide %s and %s" % (self.__class__, other.__class__))
        return state

    def __abs__(self):
        other = self.from_state(self)
        other.x = abs(other.x)
        other.y = abs(other.y)
        other.z = abs(other.z)
        other.theta = abs(other.theta)
        other.vx = abs(other.vx)
        other.vy = abs(other.vy)
        other.vz = abs(other.vz)
        other.vt = abs(other.vt)
        return other

    def __lt__(self, other):
        if isinstance(other, self.__class__):
            return (
                    self.x < other.x and
                    self.y < other.y and
                    self.z < other.z and
                    self.theta < other.theta and
                    self.vx < other.vx and
                    self.vy < other.vy and
                    self.vz < other.vz and
                    self.vt < other.vt
            )
        elif isinstance(other, int) or isinstance(other, float):
            return (
                    self.x < other and
                    self.y < other and
                    self.z < other and
                    self.theta < other and
                    self.vx < other and
                    self.vy < other and
                    self.vz < other and
                    self.vt < other
            )

    def __eq__(self, other):
        if isinstance(other, self.__class__):
            return (
                    self.x == other.x and
                    self.y == other.y and
                    self.z == other.z and
                    self.theta == other.theta and
                    self.vx == other.vx and
                    self.vy == other.vy and
                    self.vz == other.vz and
                    self.vt == other.vt
            )
        elif isinstance(other, int) or isinstance(other, float):
            return (
                    self.x == other and
                    self.y == other and
                    self.z == other and
                    self.theta == other and
                    self.vx == other and
                    self.vy == other and
                    self.vz == other and
                    self.vt == other
            )
