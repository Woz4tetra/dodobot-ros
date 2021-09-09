#!/usr/bin/env python3
import os
import cv2
import traceback
import math
import random
import time

import rospy
import rospkg

from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

from vision_msgs.msg import Detection2DArray

from db_parsing.msg import DodobotLinear
from db_parsing.msg import DodobotState
from db_parsing.msg import DodobotParallelGripper
from db_parsing.msg import DodobotTilter

from db_chassis.msg import LinearVelocity, LinearPosition

from db_parsing.srv import DodobotSetState

from db_audio.srv import PlayAudio
from db_audio.srv import StopAudio

from db_joystick.srv import SetJoystickMode, SetJoystickModeResponse

from dodobot_tools.pascal_voc import PascalVOCFrame

import ctypes
# a thread gets killed improperly within CvBridge without this causing segfaults
libgcc_s = ctypes.CDLL('libgcc_s.so.1')

from cv_bridge import CvBridge, CvBridgeError


MAX_INT32 = 0x7fffffff


NORMAL = 1
IMAGE_LABEL = 2


class ParsingJoystick:
    def __init__(self):
        self.package_name = "db_joystick"
        self.node_name = "parsing_joystick"
        rospy.init_node(
            self.node_name,
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

        self.motors_active = False
        self.play_audio_srv = None
        self.stop_audio_srv = None

        # parameters from launch file
        self.linear_axis = int(rospy.get_param("~linear_axis", 1))
        self.angular_axis = int(rospy.get_param("~angular_axis", 2))
        self.stepper_axis = int(rospy.get_param("~stepper_axis", 4))
        self.tilter_axis = int(rospy.get_param("~tilter_axis", 4))

        self.linear_scale = rospy.get_param("~linear_scale", 1.0)
        self.angular_scale = rospy.get_param("~angular_scale", 1.0)
        self.stepper_max_speed = rospy.get_param("~stepper_max_speed", 0.04424059137543106)
        self.gripper_max_dist = rospy.get_param("~gripper_max_dist", 0.08)

        self.max_tilt_speed = rospy.get_param("~max_tilt_speed", 6.0)

        self.deadzone_joy_val = rospy.get_param("~deadzone_joy_val", 0.05)
        self.joystick_topic = rospy.get_param("~joystick_topic", "/joy")

        self.joystick_mode = rospy.get_param("~joystick_mode", NORMAL)
        self.valid_modes = (NORMAL, IMAGE_LABEL)
        assert self.is_mode_valid(self.joystick_mode)

        self.detection_topic = rospy.get_param("~detection_topic", "detections")
        self.detection_dir = rospy.get_param("~detection_dir", None)
        if self.detection_dir is None:
            rospack = rospkg.RosPack()
            package_dir = rospack.get_path(self.package_name)
            self.detection_dir = package_dir + "/detections"
        if not os.path.isdir(self.detection_dir):
            os.makedirs(self.detection_dir)

        # https://play.pokemonshowdown.com/audio/cries/
        self.soundboard_right = [
            # "regirock-sound-1",
            # "regirock-sound-2",
            "chansey",
            # "porygon",
            # "porygon2",
            # "porygonz",
        ]
        self.soundboard_left = [
            "Bastion_-_15227",
            "Bastion_-_15303",
            "Bastion_-_4543",
            "Bastion_-_Beeple",
            "Bastion_-_Boo_boo_doo_de_doo",
            "Bastion_-_Bweeeeeeeeeee",
            "Bastion_-_Chirr_chirr_chirr",
            "Bastion_-_Dah-dah_weeeee",
            "Bastion_-_Doo-woo",
            "Bastion_-_Dun_dun_boop_boop",
            "Bastion_-_Dweet_dweet_dweet",
            "Bastion_-_Hee_hoo_hoo",
            "Bastion_-_Sh-sh-sh_dwee!",
            "Bastion_-_Zwee",
        ]
        
        self.soundboard_up = [
            "got thing",
            "trident",
            "summon"
            "PuzzleDone",
        ]
        self.soundboard_down = ["ring"]

        # publishing topics
        self.cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=100)
        self.linear_pub = rospy.Publisher("linear_cmd", DodobotLinear, queue_size=5)
        self.linear_vel_pub = rospy.Publisher("linear_vel_cmd", LinearVelocity, queue_size=50)
        self.linear_pos_pub = rospy.Publisher("linear_pos_cmd", LinearPosition, queue_size=50)
        self.parallel_gripper_pub = rospy.Publisher("parallel_gripper_cmd", DodobotParallelGripper, queue_size=50)
        self.tilter_pub = rospy.Publisher("tilter_cmd", DodobotTilter, queue_size=50)

        # subscription topics
        self.joy_sub = rospy.Subscriber(self.joystick_topic, Joy, self.joystick_msg_callback, queue_size=5)
        self.parallel_gripper_sub = rospy.Subscriber("parallel_gripper", DodobotParallelGripper, self.parallel_gripper_callback, queue_size=100)
        self.robot_state_sub = rospy.Subscriber("state", DodobotState, self.robot_state_callback, queue_size=100)
        self.tilter_sub = rospy.Subscriber("tilter", DodobotTilter, self.tilter_callback, queue_size=50)
        self.detections_sub = None
        if self.joystick_mode == IMAGE_LABEL:
            self.connect_to_detections()

        self.max_joy_val = 1.0
        self.prev_brake_engage_time = rospy.Time.now()

        self.twist_command.linear.x = 0.0
        self.twist_command.angular.z = 0.0

        self.gripper_dist = self.gripper_max_dist
        self.parallel_gripper_msg = DodobotParallelGripper()

        self.enable_tilt_axis = False
        self.tilt_position = 180

        self.cmd_vel_timeout = rospy.Time.now()

        self.linear_vel_command = 0.0
        self.prev_linear_vel_command = 0.0

        self.detection = None

        self.bridge = CvBridge()

        # Services
        self.set_robot_state = self.make_service_client("set_state", DodobotSetState)
        self.play_audio_srv = self.make_service_client("play_audio", PlayAudio, wait=False)
        self.stop_audio_srv = self.make_service_client("stop_audio", StopAudio, wait=False)

        self.joystick_mode_srv = rospy.Service("joystick_mode", SetJoystickMode, self.joystick_mode_callback)

        time.sleep(3.0)  # give a chance for the audio node to come up

        self.play_audio("chansey")

    def is_mode_valid(self, mode):
        return mode in self.valid_modes
        
    def joystick_mode_callback(self, req):
        if self.is_mode_valid(req.mode):
            self.joystick_mode = req.mode
            if self.joystick_mode == IMAGE_LABEL:
                self.connect_to_detections()
            elif self.joystick_mode == NORMAL:
                self.disconnect_from_detections()

            return SetJoystickModeResponse(True)
        else:
            return SetJoystickModeResponse(False)
    
    def connect_to_detections(self):
        self.detections_sub = rospy.Subscriber(self.detection_topic, Detection2DArray, self.detections_callback, queue_size=10)
    
    def disconnect_from_detections(self):
        if self.detections_sub is not None:
            self.detections_sub.unregister()
            self.detections_sub = None

    def play_audio(self, name):
        try:
            if self.play_audio_srv is not None:
                self.play_audio_srv(name)
        except rospy.ServiceException as e:
            rospy.logwarn("Failed to play audio: %s: %s" % (name, e))
    
    def stop_audio(self, name):
        try:
            if self.stop_audio_srv is not None:
                self.stop_audio_srv(name)
        except rospy.ServiceException as e:
            rospy.logwarn("Failed to play audio: %s: %s" % (name, e))
    
    def play_random_audio(self, names):
        self.play_audio(random.choice(names))

    def make_service_client(self, name, srv_type, timeout=None, wait=True):
        self.__dict__[name + "_service_name"] = name
        rospy.loginfo("Connecting to %s service" % name)
        srv_obj = rospy.ServiceProxy(name, srv_type)

        if wait:
            rospy.loginfo("Waiting for service %s" % name)
            rospy.wait_for_service(name, timeout=timeout)
            rospy.loginfo("%s service is ready" % name)
        return srv_obj

    def joy_to_speed(self, scale_factor, value):
        if abs(value) < self.deadzone_joy_val:
            return 0.0
        joy_val = abs(value) - self.deadzone_joy_val
        joy_val = math.copysign(joy_val, value)
        max_joy_val_adj = self.max_joy_val - self.deadzone_joy_val
        command = scale_factor / max_joy_val_adj * joy_val

        return command

    def is_button_down(self, msg, index):
        return msg.buttons[index] and self.did_button_change(msg, index)

    def is_button_up(self, msg, index):
        return not msg.buttons[index] and self.did_button_change(msg, index)

    def did_button_change(self, msg, index):
        return self.prev_joy_msg.buttons[index] != msg.buttons[index]

    def did_axis_change(self, msg, index):
        return self.prev_joy_msg.axes[index] != msg.axes[index]


    def joystick_msg_callback(self, msg):
        try:
            self.process_joy_msg(msg)
        except BaseException as e:
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

        # set stepper velocity and acceleration profiles
        msg = LinearPosition()
        msg.position = float("nan")
        msg.max_speed = self.stepper_max_speed
        msg.acceleration = float("nan")
        self.linear_pos_pub.publish(msg)

    def parallel_gripper_callback(self, gripper_msg):
        self.gripper_dist = gripper_msg.distance

    def toggle_gripper(self):
        if abs(self.gripper_max_dist - self.gripper_dist) < 0.005:
            self.parallel_gripper_msg.distance = 0.0
        else:
            self.parallel_gripper_msg.distance = self.gripper_max_dist

        self.parallel_gripper_pub.publish(self.parallel_gripper_msg)

    def robot_state_callback(self, msg):
        # battery_ok
        # motors_active
        # loop_rate
        # is_ready
        # robot_name
        self.motors_active = msg.motors_active

    def toggle_active(self):
        new_state = not self.motors_active
        rospy.loginfo("Setting active to %s" % new_state)
        self.set_robot_state(True, new_state)
        if new_state:
            self.play_audio("ding")

    def tilter_callback(self, msg):
        self.tilt_position = msg.position

    def set_tilter(self, position):
        msg = DodobotTilter()
        msg.command = 3
        msg.position = position
        self.tilter_pub.publish(msg)

    def toggle_tilter(self):
        msg = DodobotTilter()
        msg.command = 2
        self.tilter_pub.publish(msg)

    def process_joy_msg(self, msg):
        if self.prev_joy_msg is None:
            self.prev_joy_msg = msg
            return

        # Xbox button mapping:
        # 0: A,    1: B,     3: Y,      2: X
        # 4: L1,   5: R1,    11: menu
        # 13: L joy, 14: R joy

        # Xbox axes:
        # Lx: 0, Ly: 1
        # Rx: 2, Ry: 3
        # L brake: 5
        # R brake: 4
        # D-pad left-right: 6
        # D-pad up-down: 7
        if self.is_button_down(msg, 0):  # A
            self.home_linear()
        if self.is_button_down(msg, 1): # B
            self.toggle_gripper()
        if self.is_button_down(msg, 3): # Y
            self.toggle_active()
        if self.is_button_down(msg, 2): # X
            self.toggle_tilter()
        if self.did_button_change(msg, 4): # L1
            self.enable_tilt_axis = msg.buttons[4] == 1
            if self.enable_tilt_axis:
                self.set_linear(0.0)
            else:
                self.tilt_speed = 0
        if self.joystick_mode == IMAGE_LABEL:
            if self.did_button_change(msg, 5):
                self.save_last_detection()

        if self.did_axis_change(msg, 6):
            if msg.axes[6] > 0:  # D-pad left
                self.play_random_audio(self.soundboard_left)
            elif msg.axes[6] < 0:  # D-pad right
                self.play_random_audio(self.soundboard_right)
        if self.did_axis_change(msg, 7):
            if msg.axes[7] > 0:  # D-pad up
                self.play_random_audio(self.soundboard_up)
            elif msg.axes[7] < 0:  # D-pad down
                self.play_random_audio(self.soundboard_down)

        linear_val = self.joy_to_speed(self.linear_scale, msg.axes[self.linear_axis])
        angular_val = self.joy_to_speed(self.angular_scale, msg.axes[self.angular_axis])

        if self.enable_tilt_axis:
            self.tilt_speed = int(self.max_tilt_speed * msg.axes[self.tilter_axis])

            if self.tilt_speed != 0:
                self.set_tilter(self.tilt_position + self.tilt_speed)
        else:
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

    def detections_callback(self, msg):
        self.detection = msg

    def save_last_detection(self):
        if self.detection is None:
            rospy.logwarn("Detection is empty!")
            return
        rospy.loginfo("Saving last detection")
        
        if len(self.detection.detections) == 0:
            rospy.logwarn("Detection is has no image!")
            return

        image_msg = self.detection.detections[0].source_img
        try:
            color_image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
            return
        
        detection_timestamp = self.detection.header.stamp.to_sec()
        image_path = self.detection_dir + "/%s.jpg" % (detection_timestamp)
        detection_path = self.detection_dir + "/%s.xml" % (detection_timestamp)
        rospy.loginfo("Writing detection to %s" % detection_path)

        pascal_frame = PascalVOCFrame.from_ros_msg(self.class_mapping, color_image.shape, self.detection)
        pascal_frame.set_image_path(image_path)
        pascal_frame.write(detection_path)

        cv2.imwrite(image_path, color_image)


    def run(self):
        if not self.enabled:
            return
        clock_rate = rospy.Rate(15.0)
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
