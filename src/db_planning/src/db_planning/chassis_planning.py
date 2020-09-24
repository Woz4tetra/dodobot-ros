#!/usr/bin/python

import rospy
import tf
import math
import traceback
import actionlib

from geometry_msgs.msg import Twist

from db_planning.pid import PID
from db_planning.msg import ChassisAction, ChassisGoal, ChassisResult


# def format_exc(e):
#     tb_list = traceback.extract_tb(e.__traceback__)
#     exception_list = traceback.format_list(tb_list)
#     exception_list.insert(0, "Traceback (most recent call last):\n")
#     exception_list.append("{}: {}".format(e.__class__.__name__, str(e)))
#     exception_str = "".join(exception_list)
#     return exception_str



class ChassisPlanning:
    """
    Class definition for planning_base ROS node.
    """

    def __init__(self):
        """
        Initializes ROS and necessary services and publishers.
        """

        rospy.init_node(
            "chassis_planning"
            # disable_signals=True
            # log_level=rospy.DEBUG
        )

        self.twist_command = Twist()
        self.cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=100)

        self.tf_listener = tf.TransformListener()

        self.chassis_action_name = rospy.get_param("~chassis_action_name", "chassis_actions")
        self.chassis_server = actionlib.SimpleActionServer("/" + self.chassis_action_name, ChassisAction, self.chassis_callback, auto_start=False)
        self.chassis_server.start()
        rospy.loginfo("[%s] server started" % self.chassis_action_name)

        self.result = ChassisResult()

        self.has_active_goal = False
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_a = 0.0

        self.base_max_speed = 0.36  # m/s
        self.base_max_ang_v = 0.7  # rad/s

        self.dist_timeout_tolerance = 0.0005
        self.angle_timeout_tolerance = 0.0025
        self.move_timeout = rospy.Duration(10.0)

        self.pos_tolerance = 0.005  # m
        self.laterial_tolerance = 0.03  # m
        self.angle_tolerance = 0.005  # rad
        self.base_speed = self.base_max_speed
        self.base_ang_v = self.base_max_ang_v

        self.min_ang_v = 0.3  # rad/s
        self.min_ang_v_deadzone = 0.1

        self.min_speed = 0.05  # m/s
        self.min_speed_deadzone = 0.01

        self.dist_pid = PID(4.0, 0.5, 0.5)
        # self.lateral_pid = PID(3.0, 0.0, 0.01)
        self.angle_pid = PID(9.0, 0.0, 0.5)
        self.in_place_angle_pid = PID(24.0, 0.0, 0.1)

        self.dist_pid.set_bounds(-self.base_max_speed, self.base_max_speed)
        # self.lateral_pid.set_bounds(-self.base_max_ang_v, self.base_max_ang_v)
        self.angle_pid.set_bounds(-self.base_max_ang_v, self.base_max_ang_v)
        self.in_place_angle_pid.set_bounds(-self.base_max_ang_v, self.base_max_ang_v)

        # self.dist_pid.set_deadzones(self.min_speed_deadzone, self.min_speed)
        # self.lateral_pid.set_deadzones(self.min_ang_v_deadzone, self.min_ang_v)
        # self.angle_pid.set_deadzones(self.min_ang_v_deadzone, self.min_ang_v)
        self.in_place_angle_pid.set_deadzones(self.min_ang_v_deadzone, self.min_ang_v)


        rospy.loginfo("[%s] --- Dodobot chassis planning is up! ---" % self.chassis_action_name)


    def chassis_callback(self, goal):
        goal_x = goal.goal_x
        goal_y = goal.goal_y
        goal_angle = goal.goal_angle
        base_speed = goal.base_speed
        base_ang_v = goal.base_ang_v
        pos_tolerance = goal.pos_tolerance
        drive_forwards = goal.drive_forwards

        params = {}

        if not math.isnan(goal_x):
            params["goal_x"] = goal_x

        if not math.isnan(goal_y):
            params["goal_y"] = goal_y

        if not math.isnan(goal_angle):
            params["goal_angle"] = goal_angle

        if len(params) == 0:
            rospy.loginfo("No action required from chassis X: %s, Y: %s, A: %s %s. Skipping." % (goal_x, goal_y, goal_angle, params))
            self.set_succeeded(True)
            return

        if not math.isnan(base_speed):
            params["base_speed"] = base_speed

        if not math.isnan(base_ang_v):
            params["base_ang_v"] = base_ang_v

        if not math.isnan(pos_tolerance):
            params["pos_tolerance"] = pos_tolerance

        params["drive_forwards"] = drive_forwards

        self.tf_listener.waitForTransform("/base_link", "/odom", rospy.Time(), rospy.Duration(5.0))

        self.has_active_goal = True
        self.update_current_pose()
        rospy.loginfo("Sending parameters to chassis guidance: %s" % str(params))
        try:
            success = self.goto_pose(params)
        except BaseException as e:
            tb = traceback.format_exc()
            rospy.logerr("An error occurred while going to pose: %s" % str(tb))
            self.set_succeeded(False)
            return

        self.has_active_goal = False
        if not success:
            rospy.logerr("Failed to go to pose")

        self.set_succeeded(success)

    def set_succeeded(self, success):
        self.result.success = success
        self.chassis_server.set_succeeded(self.result)
        self.send_cmd(0.0, 0.0)


    def check_cancelled(self):
        if rospy.is_shutdown():
            self.send_cmd(0.0, 0.0)
            return True

        if self.chassis_server.is_preempt_requested():
            rospy.loginfo("%s: Preempted" % self.chassis_action_name)
            self.chassis_server.set_preempted()
            self.send_cmd(0.0, 0.0)
            return True
        else:
            return False

    def goto_pose(self, params):
        default_val = None  # float("nan")
        goal_x = params.get("goal_x", default_val)
        goal_y = params.get("goal_y", default_val)
        goal_angle = params.get("goal_angle", default_val)

        rospy.loginfo("Current pose: (%s, %s, %s). Goal pose: (%s, %s, %s)" % (
            self.current_x, self.current_y, self.current_a,
            goal_x, goal_y, goal_angle
        ))

        self.base_speed = params.get("base_speed", self.base_speed)
        self.base_ang_v = params.get("base_ang_v", self.base_ang_v)
        self.pos_tolerance = params.get("pos_tolerance", self.pos_tolerance)
        self.angle_tolerance = params.get("angle_tolerance", self.angle_tolerance)
        drive_forwards = bool(params.get("drive_forwards"))

        # if bool(goal_x is None) != bool(goal_y is None):
        #     if goal_x is None:
        #         goal_x = self.current_x
        #     if goal_y is None:
        #         goal_y = self.current_y

        self.dist_pid.set_bounds(-self.base_speed, self.base_speed)
        # self.lateral_pid.set_bounds(-self.base_ang_v, self.base_ang_v)
        self.angle_pid.set_bounds(-self.base_ang_v, self.base_ang_v)

        if goal_x is not None and goal_y is not None:
            traj_goal_angle = math.atan2(goal_y - self.current_y, goal_x - self.current_x)
            if not drive_forwards:
                traj_goal_angle += math.pi

            if not self.rotate_to_angle(traj_goal_angle, drive_forwards):
                return False
            rospy.loginfo("Initial angle rotate complete!")

            if not self.drive_to_point(goal_x, goal_y, traj_goal_angle, drive_forwards):
                return False
            rospy.loginfo("Initial drive to point complete!")

        if goal_angle is not None:
            if not self.rotate_to_angle(goal_angle):
                return False
            rospy.loginfo("Final goal angle reached!")

        rospy.loginfo("goto_pose command completed successfully!")
        return True

    def shift_reference_frame(self, x, y, angle):
        """Rotate a point to a different reference frame (applies a rotation matrix)"""
        x = x * math.cos(angle) + y * math.sin(angle)
        y = x * -math.sin(angle) + y * math.cos(angle)

        return x, y

    def convert_angle_range(self, angle):
        """Convert an angle to a -pi...pi range"""
        angle %= 2.0 * math.pi

        if angle >= math.pi:
            angle -= 2.0 * math.pi

        return angle

    def rotate_to_angle(self, angle, drive_forwards=True):
        self.in_place_angle_pid.reset()

        prev_time = rospy.Time.now()
        timeout_timer = rospy.Time.now()
        prev_error = 0.0
        rate = rospy.Rate(30.0)

        while True:
            angle_error = self.convert_angle_range(angle - self.current_a)

            if abs(angle_error) < self.angle_tolerance:
                rospy.loginfo("Goal angle current: %s, goal: %s reached!" % (self.current_a, angle))
                return True

            if abs(angle_error - prev_error) < self.angle_timeout_tolerance:
                if rospy.Time.now() - timeout_timer > self.move_timeout:
                    rospy.loginfo("Rotate command timed out after %ss" % (self.move_timeout.to_sec()))
                    return False
            else:
                timeout_timer = rospy.Time.now()
            prev_error = angle_error

            # angle_error = math.copysign(angle_error ** 2, angle_error)

            current_time = rospy.Time.now()
            dt = (current_time - prev_time).to_sec()
            prev_time = current_time

            angular_wz = self.in_place_angle_pid.update(angle_error, dt)
            self.send_cmd(0.0, angular_wz)

            rospy.loginfo("Goal: %s\tCurrent: %s\tError: %s" % (angle, self.current_a, angle_error))
            rospy.loginfo("Wz: %s" % angular_wz)

            if self.check_cancelled():
                rospy.loginfo("Drive to point routine cancelled")
                return False
            rate.sleep()

    def drive_to_point(self, goal_x, goal_y, traj_goal_angle, drive_forwards):
        self.angle_pid.reset()
        # self.lateral_pid.reset()
        self.dist_pid.reset()

        prev_dist_error = 0.0
        prev_angle_error = 0.0
        angle_timeout_timer = rospy.Time.now()
        dist_timeout_timer = rospy.Time.now()

        prev_time = rospy.Time.now()
        rate = rospy.Rate(30.0)

        while True:
            dist_error, lateral_error = self.shift_reference_frame(
                goal_x - self.current_x, goal_y - self.current_y, traj_goal_angle
            )
            if not drive_forwards:
                dist_error *= -1
            angle_error = self.convert_angle_range(traj_goal_angle - self.current_a)

            if abs(dist_error) < self.pos_tolerance and abs(angle_error) < self.angle_tolerance:  # and abs(lateral_error) < self.laterial_tolerance:
                rospy.loginfo("Goal point current: (%s, %s), goal: (%s, %s) reached!" % (self.current_x, self.current_y, goal_x, goal_y))
                self.send_cmd(0.0, 0.0)
                return True

            if abs(angle_error - prev_angle_error) < self.angle_timeout_tolerance:
                if rospy.Time.now() - angle_timeout_timer > self.move_timeout:
                    rospy.loginfo("Drive to point timed out after %ss due to stuck angle" % (self.move_timeout.to_sec()))
                    return False
            else:
                angle_timeout_timer = rospy.Time.now()
            prev_angle_error = angle_error

            if abs(dist_error - prev_dist_error) < self.angle_timeout_tolerance:
                if rospy.Time.now() - dist_timeout_timer > self.move_timeout:
                    rospy.loginfo("Drive to point timed out after %ss due to stuck distance" % (self.move_timeout.to_sec()))
                    return False
            else:
                dist_timeout_timer = rospy.Time.now()
            prev_dist_error = dist_error

            current_time = rospy.Time.now()
            dt = (current_time - prev_time).to_sec()
            prev_time = current_time

            linear_vx = self.dist_pid.update(dist_error, dt)
            if not drive_forwards:
                linear_vx *= -1.0
            # angular_wz = self.lateral_pid.update(lateral_error, dt)
            angular_wz = self.angle_pid.update(angle_error, dt)
            self.send_cmd(linear_vx, angular_wz)

            rospy.loginfo("Dist Error: %s\tAngle Error: %s" % (dist_error, angle_error))
            rospy.loginfo("Vx: %s\tWz: %s" % (linear_vx, angular_wz))

            # rospy.loginfo("Dist Error: %s\tLateral Error: %s" % (dist_error, lateral_error))
            # rospy.loginfo("Angle Goal: %s\tCurrent: %s\tError: %s" % (traj_goal_angle, self.current_a, angle_error))

            if self.check_cancelled():
                rospy.loginfo("Drive to point routine cancelled")
                return False
            rate.sleep()

    def send_cmd(self, linear_vx, angular_wz):
        self.twist_command.linear.x = linear_vx
        self.twist_command.angular.z = angular_wz
        self.cmd_vel_pub.publish(self.twist_command)

    def update_current_pose(self):
        try:
            trans, rot = self.tf_listener.lookupTransform("/odom", "/base_link", rospy.Time(0))
            self.current_x = trans[0]
            self.current_y = trans[1]
            self.current_a = tf.transformations.euler_from_quaternion(rot)[2]

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return

    def run(self):
        rate = rospy.Rate(30.0)
        while not rospy.is_shutdown():
            rate.sleep()
            if not self.has_active_goal:
                rospy.sleep(0.1)
                continue
            self.update_current_pose()



if __name__ == "__main__":
    try:
        node = ChassisPlanning()
        node.run()
        # rospy.spin()

    except rospy.ROSInterruptException:
        pass

    finally:
        rospy.loginfo("Exiting planning_base node")
