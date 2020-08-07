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

from rover6_serial_bridge.msg import Rover6Servos
from rover6_serial_bridge.srv import Rover6MenuSrv


class Rover6Teleop:
    def __init__(self):
        rospy.init_node(
            "rover6_teleop",
            disable_signals=True
            # log_level=rospy.DEBUG
        )

        # class variables
        self.twist_command = Twist()
        self.servo_command = Rover6Servos()
        self.prev_joy_msg = None

        # parameters from launch file
        self.linear_axis = int(rospy.get_param("~linear_axis", 1))
        self.angular_axis = int(rospy.get_param("~angular_axis", 2))
        self.camera_pan_axis = int(rospy.get_param("~camera_pan_axis", 3))
        self.camera_tilt_axis = int(rospy.get_param("~camera_tilt_axis", 4))

        self.linear_scale = rospy.get_param("~linear_scale", 1.0)
        self.angular_scale = rospy.get_param("~angular_scale", 1.0)

        self.deadzone_joy_val = rospy.get_param("~deadzone_joy_val", 0.05)
        self.joystick_topic = rospy.get_param("~joystick_topic", "/joy")

        # self.wheel_radius = rospy.get_param("~wheel_radius_cm", 32.5)
        # self.wheel_distance = rospy.get_param("~wheel_distance_cm", 22.0)
        # self.ticks_per_rotation = rospy.get_param("~ticks_per_rotation", 38400.0)
        # self.max_speed_cps = rospy.get_param("~max_speed_cps", 915.0)

        self.pan_right_command = rospy.get_param("~pan_right_command", 90)
        self.pan_left_command = rospy.get_param("~pan_left_command", 0)
        self.pan_center_command = rospy.get_param("~pan_center_command", 45)
        self.tilt_up_command = rospy.get_param("~tilt_up_command", 0)
        self.tilt_down_command = rospy.get_param("~tilt_down_command", 150)
        self.tilt_center_command = rospy.get_param("~tilt_center_command", 100)

        # publishing topics
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel_teleop", Twist, queue_size=100)
        self.servo_pub = rospy.Publisher("servos", Rover6Servos, queue_size=100)

        # subscription topics
        self.joy_sub = rospy.Subscriber(self.joystick_topic, Joy, self.joystick_msg_callback, queue_size=5)

        # self.tick_to_cm_factor = 2.0 * self.wheel_radius * math.pi / self.ticks_per_rotation
        # self.max_speed_mps = self.max_speed_cps / 1000.0
        # self.max_speed_tps = self.max_speed_mps / self.tick_to_cm_factor  # max speed in ticks per s

        self.max_joy_val = 1.0
        self.prev_brake_engage_time = rospy.Time.now()

        self.twist_command.linear.x = 0.0
        self.twist_command.angular.z = 0.0
        self.servo_command.camera_pan = -1
        self.servo_command.camera_tilt = -1

        # 0 = point in dircetion of joystick
        # 1 = move the turret at a certain velocity
        self.camera_command_mode = Rover6Servos.POSITION_MODE

        self.cmd_vel_timeout = rospy.Time.now()
        # rospy.Timer(rospy.Duration(0.25), self.timer_callback)

        self.menu_service_name = "rover6_menu"
        self.menu_events = {
            "up": "^",
            "down": "v",
            "left": "<",
            "right": ">",
            "enter": "e",
            "back": "b",
        }

        rospy.loginfo("Waiting for service %s" % self.menu_service_name)
        rospy.wait_for_service(self.menu_service_name)
        self.send_menu_event = rospy.ServiceProxy(self.menu_service_name, Rover6MenuSrv)
        rospy.loginfo("%s service is ready" % self.menu_service_name)

    def joy_to_speed(self, scale_factor, value):
        if abs(value) < self.deadzone_joy_val:
            return 0.0
        joy_val = abs(value) - self.deadzone_joy_val
        joy_val = math.copysign(joy_val, value)
        max_joy_val_adj = self.max_joy_val - self.deadzone_joy_val
        command = scale_factor / max_joy_val_adj * joy_val

        return command

    def joy_to_vel_servo(self, value):
        if abs(value) < self.deadzone_joy_val:
            return 0.0
        return value * 50.0

    def joy_to_servo(self, max_command, min_command, center_command, value):
        if abs(value) < self.deadzone_joy_val:
            return None
        joy_val = abs(value) - self.deadzone_joy_val
        joy_val = math.copysign(joy_val, value)
        max_joy_val_adj = self.max_joy_val - self.deadzone_joy_val

        if value > 0.0:
            command = (max_command - center_command) / max_joy_val_adj * joy_val + center_command
        else:
            command = (min_command - center_command) / -max_joy_val_adj * joy_val + center_command

        return command

    def joy_to_pan_servo(self, value):
        return self.joy_to_servo(self.pan_right_command, self.pan_left_command, self.pan_center_command, value)

    def joy_to_tilt_servo(self, value):
        return self.joy_to_servo(self.tilt_up_command, self.tilt_down_command, self.tilt_center_command, value)

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

    def set_servos(self, camera_pan_val=None, camera_tilt_val=None, mode=Rover6Servos.POSITION_MODE):
        if camera_pan_val is None:  # default position
            camera_pan_val = -1
        if camera_tilt_val is None:  # default position
            camera_tilt_val = -1

        self.servo_command.camera_pan = camera_pan_val
        self.servo_command.camera_tilt = camera_tilt_val
        if mode != self.servo_command.mode:
            if mode == Rover6Servos.POSITION_MODE:
                rospy.loginfo("Switching to camera turret to position mode")
            elif mode == Rover6Servos.VELOCITY_MODE:
                rospy.loginfo("Switching to camera turret to velocity mode")
        self.servo_command.mode = mode

    def check_brake_val(self, msg, linear_val, angular_val):
        if angular_val == 0.0 and linear_val == 0.0:
            left_brake = (msg.axes[5] + 1.0) / 2.0
            right_brake = (msg.axes[4] + 1.0) / 2.0
            if self.prev_joy_msg.axes[4] != msg.axes[4] or self.prev_joy_msg.axes[5] != msg.axes[5]:
                self.prev_brake_engage_time = rospy.Time.now()
            if rospy.Time.now() - self.prev_brake_engage_time > rospy.Duration(5.0):
                left_brake = 0.0
                right_brake = 0.0
            angular_val = self.angular_scale / 3.0 * (right_brake - left_brake)
        return angular_val

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

        linear_val = self.joy_to_speed(self.linear_scale, msg.axes[self.linear_axis])
        angular_val = self.joy_to_speed(self.angular_scale, msg.axes[self.angular_axis])
        angular_val = self.check_brake_val(msg, linear_val, angular_val)

        if self.set_twist(linear_val, angular_val):
            self.cmd_vel_pub.publish(self.twist_command)

        if self.camera_command_mode == Rover6Servos.VELOCITY_MODE:
            camera_pan_val = self.joy_to_vel_servo(msg.axes[self.camera_pan_axis])
            camera_tilt_val = -self.joy_to_vel_servo(msg.axes[self.camera_tilt_axis])
        else:  # Rover6Servos.POSITION_MODE
            camera_pan_val = self.joy_to_pan_servo(msg.axes[self.camera_pan_axis])
            camera_tilt_val = self.joy_to_tilt_servo(msg.axes[self.camera_tilt_axis])

        if self.did_button_change(msg, 0):  # A
            self.send_menu_event(self.menu_events["enter"])
        elif self.did_button_change(msg, 1):  # B
            self.send_menu_event(self.menu_events["back"])
        elif self.did_button_change(msg, 14):  # R joy
            if self.camera_command_mode == Rover6Servos.POSITION_MODE:
                self.camera_command_mode = Rover6Servos.VELOCITY_MODE
            else:
                self.camera_command_mode = Rover6Servos.POSITION_MODE

        self.set_servos(camera_pan_val, camera_tilt_val, self.camera_command_mode)
        self.servo_pub.publish(self.servo_command)

        if self.prev_joy_msg.axes[7] != msg.axes[7]:  # up d-pad
            if msg.axes[7] < 0.0:
                self.send_menu_event(self.menu_events["down"])
            elif msg.axes[7] > 0.0:
                self.send_menu_event(self.menu_events["up"])
        if self.prev_joy_msg.axes[6] != msg.axes[6]:  # down d-pad
            if msg.axes[6] < 0.0:
                self.send_menu_event(self.menu_events["left"])
            elif msg.axes[6] > 0.0:
                self.send_menu_event(self.menu_events["right"])

        self.prev_joy_msg = msg

    def timer_callback(self, event):
        if self.prev_joy_msg is None:
            return

        if event.current_real - self.prev_joy_msg.header.stamp > rospy.Duration(0.1):
            if self.set_twist(0.0, 0.0):
                self.cmd_vel_pub.publish(self.twist_command)
            if self.set_servos():
                self.servo_pub.publish(self.servo_command)
            return

        if event.current_real - self.prev_joy_msg.header.stamp > rospy.Duration(0.05):
            self.cmd_vel_pub.publish(self.twist_command)
            self.servo_pub.publish(self.servo_command)
            return

    def run(self):
        clock_rate = rospy.Rate(1)

        prev_time = rospy.get_rostime()
        while not rospy.is_shutdown():

            self.cmd_vel_pub.publish(self.twist_command)
            self.servo_pub.publish(self.servo_command)
            clock_rate.sleep()


if __name__ == "__main__":
    try:
        node = Rover6Teleop()
        # node.run()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    rospy.loginfo("Exiting earth_rover_chassis node")
