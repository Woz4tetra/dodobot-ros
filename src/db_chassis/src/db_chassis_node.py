#!/usr/bin/python
from __future__ import division
import os
import sys
import math
import datetime
import traceback

import tf
import rospy
from dynamic_reconfigure.server import Server
from geometry_msgs.msg import Twist, TwistStamped
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry

from db_chassis.cfg import DodobotChassisConfig

from db_parsing.msg import DodobotBumper
from db_parsing.msg import DodobotDrive
from db_parsing.msg import DodobotFSRs
from db_parsing.msg import DodobotGripper
from db_parsing.msg import DodobotLinear
from db_parsing.msg import DodobotTilter

from db_parsing.srv import DodobotPidSrv

print(sys.version)

class DodobotChassis:
    def __init__(self):
        rospy.init_node(
            "db_chassis",
            disable_signals=True
            # log_level=rospy.DEBUG
        )

        # rospy.on_shutdown(self.shutdown)

        # robot dimensions
        self.wheel_radius_mm = rospy.get_param("~wheel_radius_mm", 30.0)  # radius of the wheel
        # 192.05, 202.05, 197.05, 199.05
        self.wheel_distance_mm = rospy.get_param("~wheel_distance_mm", 199.05)  #  min: 172.05, max: 212.05 distance between the two wheels
        self.ticks_per_rotation = rospy.get_param("~ticks_per_rotation", 3840.0)
        self.drive_pub_name = rospy.get_param("~drive_pub_name", "drive_cmd")
        self.max_speed_tps = rospy.get_param("~max_speed_tps", 6800.0)

        self.services_enabled = rospy.get_param("~services_enabled", True)
        self.publish_odom_tf = rospy.get_param("~publish_odom_tf", True)
        self.use_sensor_msg_time = rospy.get_param("~use_sensor_msg_time", False)

        self.tilter_lower_angle = math.radians(rospy.get_param("~tilter_lower_angle_deg", -60.0))
        self.tilter_upper_angle = math.radians(rospy.get_param("~tilter_upper_angle_deg", 0.0))
        self.tilter_lower_command = rospy.get_param("~tilter_lower_command", 5)
        self.tilter_upper_command = rospy.get_param("~tilter_upper_command", 180)

        self.stepper_ticks_per_R_no_gearbox = rospy.get_param("~stepper_ticks_per_R_no_gearbox", 200.0)
        self.microsteps = rospy.get_param("~microsteps", 8.0)
        self.stepper_gearbox_ratio = rospy.get_param("~stepper_gearbox_ratio", 26.0 + 103.0 / 121.0)
        self.belt_pulley_radius_m = rospy.get_param("~belt_pulley_radius_m", 0.0121)

        self.wheel_radius_m = self.wheel_radius_mm / 1000.0
        self.wheel_distance_m = self.wheel_distance_mm / 1000.0

        self.m_to_tick_factor = self.ticks_per_rotation / (2.0 * self.wheel_radius_m * math.pi)
        self.tick_to_m_factor = 1.0 / self.m_to_tick_factor

        self.max_speed_mps = self.max_speed_tps * self.tick_to_m_factor

        self.camera_tilt_angle = self.tilter_upper_angle

        # TF parameters
        self.child_frame = rospy.get_param("~odom_child_frame", "base_link")
        self.odom_parent_frame = rospy.get_param("~odom_parent_frame", "odom")
        self.tilt_base_frame = rospy.get_param("~tilt_frame", "tilt_base_link")
        self.camera_rotate_frame = rospy.get_param("~tilt_frame", "camera_rotate_link")
        self.linear_frame = rospy.get_param("~linear_frame", "linear_link")
        self.linear_base_frame = rospy.get_param("~linear_base_frame", "linear_base_link")

        self.tf_broadcaster = tf.TransformBroadcaster()

        # Drive variables
        self.drive_sub_msg = DodobotDrive()
        self.drive_pub_msg = DodobotDrive()

        self.prev_left_ticks = None
        self.prev_right_ticks = None

        # odometry state
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_t = 0.0

        self.odom_vx = 0.0
        self.odom_vy = 0.0
        self.odom_vt = 0.0

        # linear variables
        self.stepper_ticks_per_R = self.stepper_ticks_per_R_no_gearbox * self.microsteps * self.stepper_gearbox_ratio
        self.stepper_R_per_tick = 1.0 / self.stepper_ticks_per_R
        self.step_ticks_to_linear_m = self.stepper_R_per_tick * self.belt_pulley_radius_m * 2 * math.pi
        self.zero_quat = tf.transformations.quaternion_from_euler(0.0, 0.0, 0.0)

        # Odometry message
        self.odom_msg = Odometry()
        self.odom_msg.header.frame_id = self.odom_parent_frame
        self.odom_msg.child_frame_id = self.child_frame

        # Subscribers
        self.twist_sub = rospy.Subscriber("cmd_vel", Twist, self.twist_callback, queue_size=5)
        self.drive_sub = rospy.Subscriber("drive", DodobotDrive, self.drive_callback, queue_size=100)
        self.tilter_sub = rospy.Subscriber("tilter", DodobotTilter, self.tilter_callback, queue_size=100)
        self.linear_sub = rospy.Subscriber("linear", DodobotLinear, self.linear_callback, queue_size=100)

        # Publishers
        self.odom_pub = rospy.Publisher("odom", Odometry, queue_size=5)
        self.drive_pub = rospy.Publisher(self.drive_pub_name, DodobotDrive, queue_size=100)

        # Services
        self.pid_service_name = "dodobot_pid"
        self.set_pid = None

        if self.services_enabled:
            rospy.loginfo("Waiting for service %s" % self.pid_service_name)
            rospy.wait_for_service(self.pid_service_name)
            self.set_pid = rospy.ServiceProxy(self.pid_service_name, DodobotPidSrv)
            rospy.loginfo("%s service is ready" % self.pid_service_name)

            # dynamic reconfigure
            dyn_cfg = Server(DodobotChassisConfig, lambda config, level: DodobotChassis.dynamic_callback(self, config, level))

    def dynamic_callback(self, config, level):
        if not self.services_enabled:
            rospy.logwarn("Services for this node aren't enabled!")
            return

        if self.set_pid is None:
            rospy.logwarn("%s service not ready yet!" % self.pid_service_name)
        else:
            self.call_pid_service(config)

        return config

    def call_pid_service(self, config):
        if not self.services_enabled:
            rospy.logwarn("Services for this node aren't enabled!")
        try:
            rospy.loginfo("PID service config: %s" % config)
            self.set_pid(
                config["kp_A"],
                config["ki_A"],
                config["kd_A"],
                config["kp_B"],
                config["ki_B"],
                config["kd_B"],
                config["speed_kA"],
                config["speed_kB"],
            )
        except rospy.ServiceException, e:
            rospy.logwarn("%s service call failed: %s" % (self.pid_service_name, e))

    def angle_rad_to_tilt_servo_command(self, angle_rad):
        angle_rad = angle_rad + (math.pi * 2 if angle_rad <= 0.0 else 0)  # bound to 270...360 deg
        angle_deg = math.degrees(angle_rad)
        y0 = self.tilter_lower_command
        y1 = self.tilter_upper_command
        x0 = self.tilter_lower_angle
        x1 = self.tilter_upper_angle

        servo_command = (x1 - x0) / (y1 - y0) * (angle_deg - x0) + y0
        return int(servo_command)

    def twist_callback(self, twist_msg):
        linear_speed_mps = twist_msg.linear.x  # m/s
        angular_speed_radps = twist_msg.angular.z  # rad/s

        # arc = angle * radius
        # rotation speed at the wheels
        rotational_speed_mps = angular_speed_radps * self.wheel_distance_m / 2

        left_command = self.m_to_ticks(linear_speed_mps - rotational_speed_mps)
        right_command = self.m_to_ticks(linear_speed_mps + rotational_speed_mps)

        self.drive_pub_msg.left_setpoint = left_command
        self.drive_pub_msg.right_setpoint = right_command

        self.drive_pub.publish(self.drive_pub_msg)

    def servo_to_angle(self, command, max_command, min_command, min_angle, max_angle):
        return (min_angle - max_angle) / (max_command - min_command) * (command - min_command) + max_angle

    def tilt_command_to_angle_rad(self, command):
        return self.servo_to_angle(command, self.tilter_upper_command, self.tilter_lower_command, self.tilter_upper_angle, self.tilter_lower_angle)

    def drive_callback(self, drive_sub_msg):
        self.drive_sub_msg = drive_sub_msg

        if self.prev_left_ticks == None or self.prev_right_ticks == None:
            self.prev_left_ticks = self.drive_sub_msg.left_enc_pos
            self.prev_right_ticks = self.drive_sub_msg.right_enc_pos

    def tilter_callback(self, tilter_sub_msg):
        self.camera_tilt_angle = self.tilt_command_to_angle_rad(tilter_sub_msg.position)

    def linear_callback(self, linear_sub_msg):
        stepper_z_pos = linear_sub_msg.position * self.step_ticks_to_linear_m

        now = rospy.Time.now()
        self.tf_broadcaster.sendTransform(
            (0.0, 0.0, stepper_z_pos),
            self.zero_quat,
            now,
            self.linear_frame,
            self.linear_base_frame,
        )

    def run(self):
        clock_rate = rospy.Rate(60)

        while not rospy.is_shutdown():
            try:
                # wait for encoders to be initialized
                if self.prev_left_ticks == None or self.prev_right_ticks == None:
                    continue

                self.compute_odometry()
                self.publish_chassis_data()
                self.publish_camera_tfs()
            except BaseException, e:
                traceback.print_exc()
                rospy.signal_shutdown(str(e))

            clock_rate.sleep()

    def publish_camera_tfs(self):
        now = rospy.Time.now()
        pan_tilt_quaternion = tf.transformations.quaternion_from_euler(0.0, -self.camera_tilt_angle, 0.0)
        self.tf_broadcaster.sendTransform(
            (0.0, 0.0, 0.0),
            pan_tilt_quaternion,
            now,
            self.camera_rotate_frame,
            self.tilt_base_frame
        )

    def publish_chassis_data(self):
        if self.use_sensor_msg_time:
            now = self.drive_sub_msg.header.stamp
        else:
            now = rospy.Time.now()

        odom_quaternion = tf.transformations.quaternion_from_euler(0.0, 0.0, self.odom_t)
        if self.publish_odom_tf:
            self.tf_broadcaster.sendTransform(
                (self.odom_x, self.odom_y, 0.0),
                odom_quaternion,
                now,
                self.child_frame,
                self.odom_parent_frame
            )

        self.odom_msg.header.stamp = now
        self.odom_msg.pose.pose.position.x = self.odom_x
        self.odom_msg.pose.pose.position.y = self.odom_y
        self.odom_msg.pose.pose.position.z = 0.0

        self.odom_msg.pose.pose.orientation.x = odom_quaternion[0]
        self.odom_msg.pose.pose.orientation.y = odom_quaternion[1]
        self.odom_msg.pose.pose.orientation.z = odom_quaternion[2]
        self.odom_msg.pose.pose.orientation.w = odom_quaternion[3]

        self.odom_msg.twist.twist.linear.x = self.odom_vx
        self.odom_msg.twist.twist.linear.y = self.odom_vy
        self.odom_msg.twist.twist.linear.z = 0.0

        self.odom_msg.twist.twist.angular.x = 0.0
        self.odom_msg.twist.twist.angular.y = 0.0
        self.odom_msg.twist.twist.angular.z = self.odom_vt

        self.odom_pub.publish(self.odom_msg)

    def ticks_to_m(self, ticks):
        return ticks * self.tick_to_m_factor

    def m_to_ticks(self, meters):
        return meters * self.m_to_tick_factor

    def compute_odometry(self):
        # cache values from atomic drive_sub_msg to avoid potential clashes
        cur_left_ticks = self.drive_sub_msg.left_enc_pos
        cur_right_ticks = self.drive_sub_msg.right_enc_pos
        left_enc_speed = self.drive_sub_msg.left_enc_speed
        right_enc_speed = self.drive_sub_msg.right_enc_speed

        delta_left = self.ticks_to_m(cur_left_ticks - self.prev_left_ticks)
        delta_right = self.ticks_to_m(cur_right_ticks - self.prev_right_ticks)
        delta_dist = (delta_right + delta_left) / 2

        if abs(delta_left) > 0.0001 or abs(delta_right) > 0.0001:
            left_speed = self.ticks_to_m(left_enc_speed)
            right_speed = self.ticks_to_m(right_enc_speed)
        else:
            left_speed = 0.0
            right_speed = 0.0

        # angle = arc / radius
        delta_angle = (delta_right - delta_left) / self.wheel_distance_m
        self.odom_t += delta_angle

        dx = delta_dist * math.cos(self.odom_t)
        dy = delta_dist * math.sin(self.odom_t)

        self.odom_x += dx
        self.odom_y += dy

        speed = (left_speed + right_speed) / 2
        self.odom_vx = speed * math.cos(self.odom_t)
        self.odom_vy = speed * math.sin(self.odom_t)
        self.odom_vt = (right_speed - left_speed) / self.wheel_distance_m

        # print self.odom_x, self.odom_y, math.degrees(self.odom_t)

        self.prev_left_ticks = cur_left_ticks
        self.prev_right_ticks = cur_right_ticks


if __name__ == "__main__":
    try:
        node = DodobotChassis()
        node.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        rospy.loginfo("Exiting db_chassis node")
