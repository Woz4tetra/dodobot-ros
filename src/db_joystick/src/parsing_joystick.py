#!/usr/bin/python
from __future__ import division
from __future__ import print_function
import traceback
import math

import tf
import time
import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64, Bool

from db_parsing.msg import DodobotLinear
from db_parsing.msg import DodobotParallelGripper
from db_chassis.msg import LinearVelocity

MAX_INT32 = 0x7fffffff


class ParsingJoystick:
    def __init__(self):
        rospy.init_node(
            "parsing_joystick",
            disable_signals=True
            # log_level=rospy.DEBUG
        )

        self.enabled = not rospy.get_param("joystick/enabled", True)
        if not self.enabled:  # enabled for dodobot-py
            rospy.loginfo("Joystick is not enabled. Exiting node")
            return

        # class variables
        self.twist_command = Twist()
        self.prev_joy_msg = None

        # parameters from launch file
        self.linear_axis = int(rospy.get_param("~linear_axis", 1))
        self.angular_axis = int(rospy.get_param("~angular_axis", 2))
        self.stepper_axis = int(rospy.get_param("~stepper_axis", 4))

        self.linear_scale = rospy.get_param("~linear_scale", 1.0)
        self.angular_scale = rospy.get_param("~angular_scale", 1.0)
        self.stepper_max_speed = rospy.get_param("~stepper_max_speed", 0.04424059137543106)

        self.deadzone_joy_val = rospy.get_param("~deadzone_joy_val", 0.05)
        self.joystick_topic = rospy.get_param("~joystick_topic", "/joy")


        # publishing topics
        self.cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=100)
        self.linear_pub = rospy.Publisher("linear_cmd", DodobotLinear, queue_size=100)
        self.linear_vel_pub = rospy.Publisher("linear_vel_cmd", LinearVelocity, queue_size=100)
        self.parallel_gripper_pub = rospy.Publisher("parallel_gripper_cmd", DodobotParallelGripper, queue_size=100)

        # subscription topics
        self.joy_sub = rospy.Subscriber(self.joystick_topic, Joy, self.joystick_msg_callback, queue_size=5)
        self.parallel_gripper_sub = rospy.Subscriber("parallel_gripper", DodobotParallelGripper, self.parallel_gripper_callback, queue_size=100)

        self.max_joy_val = 1.0
        self.prev_brake_engage_time = rospy.Time.now()

        self.twist_command.linear.x = 0.0
        self.twist_command.angular.z = 0.0

        self.gripper_max_dist = 0.08
        self.gripper_dist = self.gripper_max_dist
        self.parallel_gripper_msg = DodobotParallelGripper()

        self.cmd_vel_timeout = rospy.Time.now()

        self.linear_vel_command = 0.0
        self.prev_linear_vel_command = 0.0

    def joy_to_speed(self, scale_factor, value):
        if abs(value) < self.deadzone_joy_val:
            return 0.0
        joy_val = abs(value) - self.deadzone_joy_val
        joy_val = math.copysign(joy_val, value)
        max_joy_val_adj = self.max_joy_val - self.deadzone_joy_val
        command = scale_factor / max_joy_val_adj * joy_val

        return command

    def did_button_change(self, msg, index):
        return msg.buttons[index] and self.prev_joy_msg.buttons[index] != msg.buttons[index]

    def joystick_msg_callback(self, msg):
        try:
            self.process_joy_msg(msg)
        except BaseException, e:
            traceback.print_exc()
            rospy.signal_shutdown(str(e))

    def set_twist(self, linear_val, angular_val):
        if self.twist_command.linear.x != linear_val or self.twist_command.angular.z != angular_val:
            self.twist_command.linear.x = linear_val
            self.twist_command.angular.z = angular_val
            self.cmd_vel_timeout = rospy.Time.now()
            return True
        else:
            return rospy.Time.now() - self.cmd_vel_timeout < rospy.Duration(0.5)

    def set_linear(self, value):
        self.linear_vel_command = self.stepper_max_speed * value
        if self.linear_vel_command != self.prev_linear_vel_command:
            self.prev_linear_vel_command = self.linear_vel_command
            msg = LinearVelocity()

            msg.velocity = self.linear_vel_command
            msg.acceleration = float("nan")

            self.linear_vel_pub.publish(msg)

    def home_linear(self):
        msg = DodobotLinear()
        msg.command_type = 4  # home stepper command
        msg.max_speed = -1
        msg.acceleration = -1
        self.linear_pub.publish(msg)

    def parallel_gripper_callback(self, gripper_msg):
        self.gripper_dist = gripper_msg.distance

    def toggle_gripper(self):
        if abs(self.gripper_dist) < 0.005:
            self.parallel_gripper_msg.distance = self.gripper_max_dist
        else:
            self.parallel_gripper_msg.distance = 0.0

        self.parallel_gripper_pub.publish(self.parallel_gripper_msg)

    def process_joy_msg(self, msg):
        if self.prev_joy_msg is None:
            self.prev_joy_msg = msg
            return

        # Xbox button mapping:
        # 0: A,    1: B,     3: X,      4: Y
        # 6: L1,   7: R1,    6: Select, 7: Start
        # 11: Menu, 13: L joy, 14: R joy

        # Xbox axes:
        # Lx: 0, Ly: 1
        # Rx: 2, Ry: 3
        # L brake: 5
        # R brake: 4
        # D-pad left-right: 6
        # D-pad up-down: 7
        if self.did_button_change(msg, 0):  # A
            self.home_linear()
        elif self.did_button_change(msg, 1): # B
            self.toggle_gripper()

        linear_val = self.joy_to_speed(self.linear_scale, msg.axes[self.linear_axis])
        angular_val = self.joy_to_speed(self.angular_scale, msg.axes[self.angular_axis])

        self.set_linear(msg.axes[self.stepper_axis])

        if self.set_twist(linear_val, angular_val):
            self.cmd_vel_pub.publish(self.twist_command)

        self.prev_joy_msg = msg

    def timer_callback(self, event):
        if self.prev_joy_msg is None:
            return

        if event.current_real - self.prev_joy_msg.header.stamp > rospy.Duration(0.1):
            if self.set_twist(0.0, 0.0):
                self.cmd_vel_pub.publish(self.twist_command)
            return

        if event.current_real - self.prev_joy_msg.header.stamp > rospy.Duration(0.05):
            self.cmd_vel_pub.publish(self.twist_command)
            return

    def run(self):
        if not self.enabled:
            return
        clock_rate = rospy.Rate(15.0)

        prev_time = rospy.get_rostime()
        while not rospy.is_shutdown():

            self.cmd_vel_pub.publish(self.twist_command)
            clock_rate.sleep()


if __name__ == "__main__":
    try:
        node = ParsingJoystick()
        # node.run()
        if node.enabled:
            rospy.spin()
    except rospy.ROSInterruptException:
        pass
    rospy.loginfo("Exiting parsing_joystick node")
