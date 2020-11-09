#!/usr/bin/python
from __future__ import division
from __future__ import print_function
import os
import sys
import math
import datetime
import traceback

import tf
import rospy
import numpy as np
from scipy.interpolate import interp1d

from dynamic_reconfigure.server import Server
from geometry_msgs.msg import Twist, TwistStamped
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState

from db_chassis.cfg import DodobotChassisConfig
from db_chassis.srv import DodobotOdomReset, DodobotOdomResetResponse
from db_chassis.msg import LinearPosition
from db_chassis.msg import LinearVelocity

from db_parsing.msg import DodobotBumper
from db_parsing.msg import DodobotDrive
from db_parsing.msg import DodobotFSRs
from db_parsing.msg import DodobotGripper
from db_parsing.msg import DodobotParallelGripper
from db_parsing.msg import DodobotLinear
from db_parsing.msg import DodobotTilter

from db_parsing.srv import DodobotPidSrv

from geometric_estimator import GeometricEstimator
from runge_kutta_estimator import RungeKuttaEstimator


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
        self.wheel_distance_mm = rospy.get_param("~wheel_distance_mm", 183.21)  #  min: 172.05, max: 212.05 distance between the two wheels
        self.ticks_per_rotation = rospy.get_param("~ticks_per_rotation", 3840.0)
        self.drive_pub_name = rospy.get_param("~drive_pub_name", "drive_cmd")
        self.max_speed_tps = rospy.get_param("~max_speed_tps", 9000.0)

        self.min_angular_speed = rospy.get_param("~min_angular_speed", 0.0)
        self.max_angular_speed = rospy.get_param("~max_angular_speed", 0.0)
        self.min_linear_speed = rospy.get_param("~min_linear_speed", 0.0)
        self.max_linear_speed = rospy.get_param("~max_linear_speed", 0.0)
        self.zero_speed_epsilon = rospy.get_param("~zero_speed_epsilon", 0.01)

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

        self.gripper_open_cmd = rospy.get_param("~gripper_open_cmd", 50)  # 50
        self.gripper_closed_cmd = rospy.get_param("~gripper_closed_cmd", 162)  # 157
        self.gripper_open_angle = rospy.get_param("~gripper_open_angle", 344.89282)
        self.gripper_closed_angle = rospy.get_param("~gripper_closed_angle", 302.560457)

        self.gripper_open_angle = math.radians(self.gripper_open_angle)
        self.gripper_closed_angle = math.radians(self.gripper_closed_angle)

        self.armature_length = rospy.get_param("~armature_length", 0.06)
        self.armature_width = rospy.get_param("~armature_width", 0.01)
        self.hinge_pin_to_armature_end = rospy.get_param("~hinge_pin_to_armature_end", 0.004)
        self.hinge_pin_diameter = rospy.get_param("~hinge_pin_diameter", 0.0028575)

        self.hinge_pin_to_pad_plane = rospy.get_param("~hinge_pin_to_pad_plane", 0.0055)
        self.pad_extension_offset = rospy.get_param("~pad_extension_offset", 0.01998285)
        self.central_axis_dist = rospy.get_param("~central_axis_dist", 0.015)

        self.rotation_offset_x = self.armature_length + self.hinge_pin_to_armature_end
        self.rotation_offset_y = (self.armature_width + self.hinge_pin_diameter) / 2.0

        self.parallel_dist_interp_fn = None
        self.compute_parallel_dist_to_angle_lookup()

        # TF parameters
        self.child_frame = rospy.get_param("~odom_child_frame", "base_link")
        self.odom_parent_frame = rospy.get_param("~odom_parent_frame", "odom")
        self.tilt_base_frame = rospy.get_param("~tilt_frame", "tilt_base_link")
        self.camera_rotate_frame = rospy.get_param("~tilt_frame", "camera_rotate_link")
        self.linear_frame = rospy.get_param("~linear_frame", "linear_link")
        self.linear_base_frame = rospy.get_param("~linear_base_frame", "linear_base_link")

        self.tf_broadcaster = tf.TransformBroadcaster()

        # Odometry estimators
        # methods: geometric, runge-kutta
        self.odom_estimate_method = rospy.get_param("~odom_estimate_method", "geometric")
        self.odom_estimate_fn = None
        if self.odom_estimate_method == "geometric":
            self.odom_estimator = GeometricEstimator(self.wheel_distance_m)
        elif self.odom_estimate_method == "runge-kutta":
            self.odom_estimator = RungeKuttaEstimator(self.wheel_distance_m)
        else:
            raise ValueError("'odom_estimate_method' is not geometric or runge-kutta")

        # self.comparison_estimators = {
        #     "geometric": GeometricEstimator(self.wheel_distance_m),
        #     # "runge-kutta": RungeKuttaEstimator(self.wheel_distance_m)
        # }


        # Drive variables
        self.drive_sub_msg = DodobotDrive()
        self.drive_pub_msg = DodobotDrive()

        self.drive_sub_msg_dt_sum = 0.0
        self.drive_sub_msg_dt_sum_count = 0

        self.prev_left_ticks = None
        self.prev_right_ticks = None
        self.prev_drive_msg_time = rospy.Time.now()
        self.prev_odom_time = rospy.Time.now()

        # gripper variables
        self.parallel_gripper_msg = DodobotParallelGripper()
        self.gripper_msg = DodobotGripper()

        # odometry state
        self.odom_timestamp = rospy.Time.now()
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_t = 0.0

        self.odom_vx = 0.0
        self.odom_vy = 0.0
        self.odom_vt = 0.0
        self.odom_speed = 0.0

        self.odom_covariance = [
            1e-3, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 1e-3, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 1e-3, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 1e-3, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 1e-3, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 1e-3
        ]

        self.linear_speed_cmd = 0.0
        self.angular_speed_cmd = 0.0

        # linear variables
        self.stepper_ticks_per_R = self.stepper_ticks_per_R_no_gearbox * self.microsteps * self.stepper_gearbox_ratio
        self.stepper_R_per_tick = 1.0 / self.stepper_ticks_per_R
        self.step_ticks_to_linear_m = self.stepper_R_per_tick * self.belt_pulley_radius_m * 2 * math.pi
        self.step_linear_m_to_ticks = 1.0 / self.step_ticks_to_linear_m

        # velocity and acceleration commands at the microcontroller side are
        # 4 orders of magnitude larger for some reason...
        self.step_linear_m_to_speed_ticks = self.step_linear_m_to_ticks * 1E4
        self.zero_quat = tf.transformations.quaternion_from_euler(0.0, 0.0, 0.0)

        # Odometry message
        self.odom_msg = Odometry()
        self.odom_msg.header.frame_id = self.odom_parent_frame
        self.odom_msg.child_frame_id = self.child_frame

        # JointState messages
        self.linear_joint = JointState()
        self.linear_joint.header.frame_id = "base_link"
        self.linear_joint.name = ["linear_to_linear_base_link"]
        self.linear_joint.position = [0.0]
        self.linear_joint.velocity = []
        self.linear_joint.effort = []
        self.linear_joint_pub = rospy.Publisher("linear_joint_state", JointState, queue_size=10)

        self.tilter_joint = JointState()
        self.tilter_joint.header.frame_id = "base_link"
        self.tilter_joint.name = ["tilt_base_to_camera_rotate_joint"]
        self.tilter_joint.position = [0.0]
        self.tilter_joint.velocity = []
        self.tilter_joint.effort = []
        self.tilter_joint_pub = rospy.Publisher("tilter_joint_state", JointState, queue_size=10)

        # Publishers
        self.odom_pub = rospy.Publisher("odom", Odometry, queue_size=5)
        self.drive_pub = rospy.Publisher(self.drive_pub_name, DodobotDrive, queue_size=100)
        self.gripper_pub = rospy.Publisher("gripper_cmd", DodobotGripper, queue_size=100)
        self.parallel_gripper_pub = rospy.Publisher("parallel_gripper", DodobotParallelGripper, queue_size=100)
        self.linear_pub = rospy.Publisher("linear_cmd", DodobotLinear, queue_size=100)
        self.linear_pos_pub = rospy.Publisher("linear_pos", LinearPosition, queue_size=100)

        # Subscribers
        self.twist_sub = rospy.Subscriber("cmd_vel", Twist, self.twist_callback, queue_size=5)
        self.drive_sub = rospy.Subscriber("drive", DodobotDrive, self.drive_callback, queue_size=100)
        self.tilter_sub = rospy.Subscriber("tilter", DodobotTilter, self.tilter_callback, queue_size=100)
        self.linear_sub = rospy.Subscriber("linear", DodobotLinear, self.linear_callback, queue_size=100)
        self.gripper_sub = rospy.Subscriber("gripper", DodobotGripper, self.gripper_callback, queue_size=100)
        self.parallel_gripper_sub = rospy.Subscriber("parallel_gripper_cmd", DodobotParallelGripper, self.parallel_gripper_callback, queue_size=100)
        self.linear_pos_sub = rospy.Subscriber("linear_pos_cmd", LinearPosition, self.linear_pos_callback, queue_size=100)
        self.linear_vel_sub = rospy.Subscriber("linear_vel_cmd", LinearVelocity, self.linear_vel_callback, queue_size=100)

        # Services
        self.pid_service_name = "dodobot_pid"
        self.set_pid = None
        self.first_time_pid_setup = False

        self.odom_reset_service_name = "dodobot_odom_reset"
        self.odom_reset = None

        if self.services_enabled:
            rospy.loginfo("Waiting for service %s" % self.pid_service_name)
            rospy.wait_for_service(self.pid_service_name)
            self.set_pid = rospy.ServiceProxy(self.pid_service_name, DodobotPidSrv)
            rospy.loginfo("%s service is ready" % self.pid_service_name)

            # dynamic reconfigure
            dyn_cfg = Server(DodobotChassisConfig, lambda config, level: DodobotChassis.dynamic_callback(self, config, level))

            rospy.loginfo("Setting up service %s" % self.odom_reset_service_name)
            self.odom_reset = rospy.Service(self.odom_reset_service_name, DodobotOdomReset, self.odom_reset_callback)
            rospy.loginfo("%s service is ready" % self.odom_reset_service_name)

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
        if not self.first_time_pid_setup:
            self.first_time_pid_setup = True
            return
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

    def odom_reset_callback(self, req):
        self.odom_x = req.x
        self.odom_y = req.y
        self.odom_t = req.t

        self.odom_estimator.reset(self.odom_x, self.odom_y, self.odom_t)

        # for estimator in self.comparison_estimators.values():
        #     estimator.reset(self.odom_x, self.odom_y, self.odom_t)

        rospy.loginfo("Resetting odom to x: %s, y: %s, a: %s" % (self.odom_x, self.odom_y, self.odom_t))
        return DodobotOdomResetResponse(True)

    def angle_rad_to_tilt_servo_command(self, angle_rad):
        angle_rad = angle_rad + (math.pi * 2 if angle_rad <= 0.0 else 0)  # bound to 270...360 deg
        angle_deg = math.degrees(angle_rad)
        y0 = self.tilter_lower_command
        y1 = self.tilter_upper_command
        x0 = self.tilter_lower_angle
        x1 = self.tilter_upper_angle

        servo_command = (x1 - x0) / (y1 - y0) * (angle_deg - x0) + y0
        return int(servo_command)

    def bound_speed(self, value, lower, upper, epsilon):
        abs_value = abs(value)
        if abs_value < lower:
            if abs_value > epsilon:
                value = math.copysign(lower, value)
            else:
                value = 0.0
        if upper != 0.0 and abs_value > upper:
            value = math.copysign(upper, value)
        return value

    def twist_callback(self, twist_msg):
        linear_speed_mps = twist_msg.linear.x  # m/s
        angular_speed_radps = twist_msg.angular.z  # rad/s

        linear_speed_mps = self.bound_speed(linear_speed_mps, self.min_linear_speed, self.max_linear_speed, self.zero_speed_epsilon)
        angular_speed_radps = self.bound_speed(
            angular_speed_radps,
            self.min_angular_speed if linear_speed_mps == 0.0 else 0.0,
            self.max_angular_speed,
            self.zero_speed_epsilon if linear_speed_mps == 0.0 else 0.0)

        self.linear_speed_cmd = linear_speed_mps
        self.angular_speed_cmd = angular_speed_radps

        # arc = angle * radius
        # rotation speed at the wheels
        rotational_speed_mps = angular_speed_radps * self.wheel_distance_m / 2

        left_command = self.m_to_ticks(linear_speed_mps - rotational_speed_mps)
        right_command = self.m_to_ticks(linear_speed_mps + rotational_speed_mps)

        larger_cmd = max(left_command, right_command)
        if abs(larger_cmd) > self.max_speed_tps:
            abs_left = abs(left_command)
            abs_right = abs(right_command)
            if abs_left > abs_right:
                left_command = math.copysign(self.max_speed_tps, left_command)
                right_command = math.copysign(self.max_speed_tps * abs_right / abs_left, right_command)
            else:
                left_command = math.copysign(self.max_speed_tps * abs_left / abs_right, left_command)
                right_command = math.copysign(self.max_speed_tps, right_command)

        self.drive_pub_msg.left_setpoint = left_command
        self.drive_pub_msg.right_setpoint = right_command

        self.drive_pub.publish(self.drive_pub_msg)

    def servo_to_angle(self, command, max_command, min_command, min_angle, max_angle):
        return (max_angle - min_angle) / (max_command - min_command) * (float(command) - min_command) + min_angle

    def angle_to_servo(self, command, max_command, min_command, min_angle, max_angle):
        return int((max_command - min_command) / (max_angle - min_angle) * (command - min_angle) + min_command)

    def tilt_command_to_angle_rad(self, command):
        return self.servo_to_angle(command, self.tilter_upper_command, self.tilter_lower_command, self.tilter_lower_angle, self.tilter_upper_angle)

    def drive_callback(self, drive_sub_msg):
        # dt = (drive_sub_msg.header.stamp - self.prev_drive_msg_time).to_sec()
        # self.drive_sub_msg_dt_sum += dt
        # self.drive_sub_msg_dt_sum_count += 1
        # if self.drive_sub_msg_dt_sum_count > 100:
        #     rospy.loginfo("drive dt avg: %s" % (
        #             self.drive_sub_msg_dt_sum / self.drive_sub_msg_dt_sum_count
        #         )
        #     )
        #     self.drive_sub_msg_dt_sum = 0.0
        #     self.drive_sub_msg_dt_sum_count = 0
        # self.prev_drive_msg_time = drive_sub_msg.header.stamp
        
        self.drive_sub_msg = drive_sub_msg

        if self.prev_left_ticks == None or self.prev_right_ticks == None:
            self.prev_left_ticks = self.drive_sub_msg.left_enc_pos
            self.prev_right_ticks = self.drive_sub_msg.right_enc_pos

    def tilter_callback(self, tilter_sub_msg):
        self.camera_tilt_angle = self.tilt_command_to_angle_rad(tilter_sub_msg.position)
        self.tilter_joint.position[0] = -self.camera_tilt_angle

    def linear_callback(self, linear_sub_msg):
        stepper_z_pos = linear_sub_msg.position * self.step_ticks_to_linear_m

        now = rospy.Time.now()
        # self.tf_broadcaster.sendTransform(
        #     (0.0, 0.0, stepper_z_pos),
        #     self.zero_quat,
        #     now,
        #     self.linear_frame,
        #     self.linear_base_frame,
        # )

        self.linear_joint.position[0] = stepper_z_pos
        self.linear_joint.header.stamp = now

        msg = LinearPosition()
        msg.position = stepper_z_pos
        self.linear_pos_pub.publish(msg)

    def linear_pos_callback(self, msg):
        linear_msg = DodobotLinear()

        linear_msg.command_type = 0
        linear_msg.command_value = int(msg.position * self.step_linear_m_to_ticks)

        if math.isnan(msg.max_speed):
            linear_msg.max_speed = -1
        else:
            linear_msg.max_speed = int(msg.max_speed * self.step_linear_m_to_speed_ticks)

        if math.isnan(msg.acceleration):
            linear_msg.acceleration = -1
        else:
            linear_msg.acceleration = int(msg.acceleration * self.step_linear_m_to_speed_ticks)

        self.linear_pub.publish(linear_msg)

    def linear_vel_callback(self, msg):
        linear_msg = DodobotLinear()
        linear_msg.command_type = 1
        linear_msg.command_value = int(msg.velocity * self.step_linear_m_to_speed_ticks)
        linear_msg.max_speed = linear_msg.command_value

        if math.isnan(msg.acceleration):
            linear_msg.acceleration = -1
        else:
            linear_msg.acceleration = int(msg.acceleration * self.step_linear_m_to_speed_ticks)

        self.linear_pub.publish(linear_msg)

    def parallel_gripper_callback(self, parallel_gripper_msg):
        angle = self.parallel_dist_to_angle(parallel_gripper_msg.distance)
        servo_pos = self.angle_to_servo(angle, self.gripper_closed_cmd, self.gripper_open_cmd, self.gripper_closed_angle, self.gripper_open_angle)

        self.gripper_msg.header.stamp = rospy.Time.now()
        self.gripper_msg.position = servo_pos
        self.gripper_msg.force_threshold = -1  # TODO: add force calculations
        # if math.isnan(parallel_gripper_msg.force_threshold):
        #     self.gripper_msg.force_threshold = -1
        # else:
        #     pass

        self.gripper_pub.publish(self.gripper_msg)

    def gripper_callback(self, gripper_msg):
        angle = self.servo_to_angle(gripper_msg.position, self.gripper_closed_cmd, self.gripper_open_cmd, self.gripper_closed_angle, self.gripper_open_angle)
        parallel_dist = self.angle_to_parallel_dist(angle)
        self.parallel_gripper_msg.header.stamp = rospy.Time.now()
        self.parallel_gripper_msg.distance = parallel_dist

        self.parallel_gripper_pub.publish(self.parallel_gripper_msg)

    def angle_to_parallel_dist(self, angle_rad):
        angle_adj = angle_rad - math.pi*2 + math.pi/2
        hinge_pin_x = self.rotation_offset_x * math.cos(angle_adj) - self.rotation_offset_y * math.sin(angle_adj)
        parallel_dist = 2.0 * (hinge_pin_x - self.hinge_pin_to_pad_plane - self.pad_extension_offset + self.central_axis_dist)
        return parallel_dist

    def compute_parallel_dist_to_angle_lookup(self):
        angle_buffer = math.radians(20.0)
        angle_sample_start = self.gripper_open_angle + angle_buffer
        angle_sample_stop = self.gripper_closed_angle - angle_buffer
        angle_samples = np.linspace(angle_sample_start, angle_sample_stop, 100)
        dist_samples = []
        for angle in angle_samples:
            parallel_dist = self.angle_to_parallel_dist(angle)
            dist_samples.append(parallel_dist)
        self.parallel_dist_interp_fn = interp1d(dist_samples, angle_samples, kind="linear")

    def parallel_dist_to_angle(self, parallel_dist):
        return self.parallel_dist_interp_fn(parallel_dist)

    def run(self):
        clock_rate = rospy.Rate(60)

        # prev_report_time = rospy.Time.now()
        try:
            while not rospy.is_shutdown():
                # wait for encoders to be initialized
                if self.prev_left_ticks == None or self.prev_right_ticks == None:
                    continue

                self.compute_odometry()
                self.publish_chassis_data()
                self.publish_joint_states()
                # self.publish_camera_tfs()

                # if rospy.Time.now() - prev_report_time > rospy.Duration(0.25):
                #     rospy.loginfo("odom; speed: %s\tangular: %s" % (self.odom_speed, self.odom_vt))
                #     rospy.loginfo("cmd; speed: %s\tangular: %s" % (self.linear_speed_cmd, self.angular_speed_cmd))
                #     rospy.loginfo("setpoint; l: %s\tr: %s" % (self.drive_pub_msg.left_setpoint, self.drive_pub_msg.right_setpoint))
                #     rospy.loginfo("measure; l: %s\tr: %s\n" % (self.drive_sub_msg.left_enc_speed, self.drive_sub_msg.right_enc_speed))
                #
                #     prev_report_time = rospy.Time.now()
                clock_rate.sleep()
        except BaseException, e:
            traceback.print_exc()
            rospy.signal_shutdown(str(e))

    def publish_joint_states(self):
        self.linear_joint_pub.publish(self.linear_joint)
        self.tilter_joint_pub.publish(self.tilter_joint)

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
            now = self.odom_timestamp

        # rospy.loginfo("drive t: %s, odom t: %s, dt: %s" % (
        #     self.drive_sub_msg.header.stamp.to_sec(),
        #     now.to_sec(),
        #     (now - self.drive_sub_msg.header.stamp).to_sec()
        #     )
        # )

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

        self.odom_msg.pose.covariance = self.odom_covariance

        self.odom_pub.publish(self.odom_msg)

    def ticks_to_m(self, ticks):
        return ticks * self.tick_to_m_factor

    def m_to_ticks(self, meters):
        return meters * self.m_to_tick_factor

    def compute_odometry(self):
        now = rospy.Time.now()
        self.odom_timestamp = now
        dt = (now - self.prev_odom_time).to_sec()
        self.prev_odom_time = now

        cur_left_ticks = self.drive_sub_msg.left_enc_pos
        cur_right_ticks = self.drive_sub_msg.right_enc_pos
        delta_left = self.ticks_to_m(cur_left_ticks - self.prev_left_ticks)
        delta_right = self.ticks_to_m(cur_right_ticks - self.prev_right_ticks)

        left_enc_speed = self.drive_sub_msg.left_enc_speed
        right_enc_speed = self.drive_sub_msg.right_enc_speed
        left_speed = self.ticks_to_m(left_enc_speed)
        right_speed = self.ticks_to_m(right_enc_speed)

        self.prev_left_ticks = cur_left_ticks
        self.prev_right_ticks = cur_right_ticks

        self.odom_estimator.update(
            delta_left=delta_left,
            delta_right=delta_right,
            left_speed=left_speed,
            right_speed=right_speed,
            dt=dt,
        )

        # if len(self.comparison_estimators) > 0:
        #     rospy.loginfo("main estimator: %s" % self.odom_estimator)
        #     for name, estimator in self.comparison_estimators.items():
        #         estimator.update(
        #             delta_left=delta_left,
        #             delta_right=delta_right,
        #             left_speed=left_speed,
        #             right_speed=right_speed,
        #             dt=dt,
        #         )
        #         rospy.loginfo("%s: %s" % (name, estimator))

        self.odom_x = self.odom_estimator.x
        self.odom_y = self.odom_estimator.y
        self.odom_t = self.odom_estimator.theta

        self.odom_speed = self.odom_estimator.v
        self.odom_vt = self.odom_estimator.w

        self.odom_vx = self.odom_estimator.vx
        self.odom_vy = self.odom_estimator.vy

if __name__ == "__main__":
    try:
        node = DodobotChassis()
        node.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        rospy.loginfo("Exiting db_chassis node")
