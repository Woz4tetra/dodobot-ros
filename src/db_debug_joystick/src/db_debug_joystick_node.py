#!/usr/bin/env python3
import time
import rospy
import random

from sensor_msgs.msg import Joy

from geometry_msgs.msg import Twist

from std_msgs.msg import ColorRGBA

from db_parsing.msg import DodobotLinear
from db_parsing.msg import DodobotState
from db_parsing.msg import DodobotParallelGripper
from db_parsing.msg import DodobotTilter

from db_chassis.msg import LinearVelocity, LinearPosition

from db_parsing.srv import DodobotSetState

from db_audio.srv import PlayAudio
from db_audio.srv import StopAudio

from db_tools.joystick import Joystick


class DodobotDebugJoystick:
    def __init__(self):
        rospy.init_node(
            "db_debug_joystick",
            # disable_signals=True
            # log_level=rospy.DEBUG
        )

        self.twist_command = Twist()
        self.twist_command.linear.x = 0.0
        self.twist_command.linear.y = 0.0
        self.twist_command.angular.z = 0.0

        self.cmd_vel_timer = rospy.Time.now()

        self.cmd_vel_timeout = rospy.Duration(0.5)
        self.send_timeout = rospy.Duration(self.cmd_vel_timeout.to_sec() + 1.0)
        self.disable_timeout = rospy.Duration(30.0)

        assert self.send_timeout.to_sec() > self.cmd_vel_timeout.to_sec()

        # parameters from launch file
        self.linear_axis = rospy.get_param("~linear_axis", "left/Y").split("/")
        self.angular_axis = rospy.get_param("~angular_axis", "left/X").split("/")
        self.stepper_axis = rospy.get_param("~stepper_axis", "right/Y").split("/")
        self.tilter_axis = rospy.get_param("~tilter_axis", "right/Y").split("/")

        self.stepper_max_speed = rospy.get_param("~stepper_max_speed", 0.04424059137543106)
        self.gripper_max_dist = rospy.get_param("~gripper_max_dist", 0.08)

        self.max_tilt_speed = rospy.get_param("~max_tilt_speed", 6.0)

        self.linear_scale = float(rospy.get_param("~linear_scale", 1.0))
        self.angular_scale = float(rospy.get_param("~angular_scale", 1.0))

        self.deadzone_joy_val = float(rospy.get_param("~deadzone_joy_val", 0.05))
        self.joystick_topic = rospy.get_param("~joystick_topic", "/joy")

        self.button_mapping = rospy.get_param("~button_mapping", None)
        assert self.button_mapping is not None
        self.axis_mapping = rospy.get_param("~axis_mapping", None)
        assert self.axis_mapping is not None

        self.gripper_dist = self.gripper_max_dist
        self.parallel_gripper_msg = DodobotParallelGripper()

        self.enable_tilt_axis = False
        self.tilt_position = 180

        self.linear_vel_command = 0.0
        self.prev_linear_vel_command = 0.0

        self.led_pattern_index = 0
        self.led_colors = [
            ColorRGBA(0.0, 0.0, 0.0, 0.0),
            ColorRGBA(1.0, 1.0, 1.0, 1.0),
            ColorRGBA(1.0, 0.0, 0.0, 0.0),
            ColorRGBA(0.0, 1.0, 0.0, 0.0),
            ColorRGBA(0.0, 0.0, 1.0, 0.0),
        ]

        self.joystick = Joystick(self.button_mapping, self.axis_mapping)

        # publishing topics
        self.cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
        self.linear_pub = rospy.Publisher("linear_cmd", DodobotLinear, queue_size=5)
        self.linear_vel_pub = rospy.Publisher("linear_vel_cmd", LinearVelocity, queue_size=10)
        self.linear_pos_pub = rospy.Publisher("linear_pos_cmd", LinearPosition, queue_size=10)
        self.parallel_gripper_pub = rospy.Publisher("parallel_gripper_cmd", DodobotParallelGripper, queue_size=10)
        self.tilter_pub = rospy.Publisher("tilter_cmd", DodobotTilter, queue_size=10)
        self.led_pub = rospy.Publisher("ring_light", ColorRGBA, queue_size=5)

        self.motors_active = False
        self.play_audio_srv = None
        self.stop_audio_srv = None

        # https://play.pokemonshowdown.com/audio/cries/
        self.soundboard_down = [
            # "regirock-sound-1",
            # "regirock-sound-2",
            "chansey",
            "porygon",
            "porygon2",
            "porygonz",
        ]
        self.soundboard_up = [
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
        
        # self.soundboard_up = [
        #     "got thing",
        #     "trident",
        #     "summon"
        #     "PuzzleDone",
        # ]
        # self.soundboard_down = ["ring"]

        # subscription topics
        self.joy_sub = rospy.Subscriber(self.joystick_topic, Joy, self.joystick_msg_callback, queue_size=5)
        self.parallel_gripper_sub = rospy.Subscriber("parallel_gripper", DodobotParallelGripper, self.parallel_gripper_callback, queue_size=100)
        self.robot_state_sub = rospy.Subscriber("state", DodobotState, self.robot_state_callback, queue_size=100)
        self.tilter_sub = rospy.Subscriber("tilter", DodobotTilter, self.tilter_callback, queue_size=50)

        # Services
        self.set_robot_state = self.make_service_client("set_state", DodobotSetState)
        self.play_audio_srv = self.make_service_client("play_audio", PlayAudio, wait=False)
        self.stop_audio_srv = self.make_service_client("stop_audio", StopAudio, wait=False)
        # time.sleep(3.0)  # give a chance for the audio node to come up
        # self.play_audio("chansey")

        rospy.loginfo("Debug joystick is ready!")

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

    def select_led_pattern(self, index):
        self.led_pattern_index = index % len(self.led_colors)
        self.led_pub.publish(self.led_colors[self.led_pattern_index])

    def make_service_client(self, name, srv_type, timeout=None, wait=True):
        self.__dict__[name + "_service_name"] = name
        rospy.loginfo("Connecting to %s service" % name)
        srv_obj = rospy.ServiceProxy(name, srv_type)

        if wait:
            rospy.loginfo("Waiting for service %s" % name)
            rospy.wait_for_service(name, timeout=timeout)
            rospy.loginfo("%s service is ready" % name)
        return srv_obj

    def robot_state_callback(self, msg):
        # battery_ok
        # motors_active
        # loop_rate
        # is_ready
        # robot_name
        self.motors_active = msg.motors_active

    def set_motors_active(self, new_state):
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

    def joystick_msg_callback(self, msg):
        """
        If the joystick disconnects, this callback will stop being called.
        If a non-zero twist is set, the command timer will be reset
            This way, if the joystick is idle, it will stop publishing after send timer is exceeded
        If the command timer is exceeded in the main loop, the twist command will be set to zero
            If the joystick disconnects (i.e. this callback stops being called), the twist command will
            quickly be set to zero
        If the send timer is exceeded in the main loop, this node will stop publishing twist
        Note: the idle axis (one of the triggers) can be pressed to keep the command timer active so the robot doesn't disable itself
        """

        self.joystick.update(msg)
        
        if self.joystick.did_button_down(("main", "B")):
            self.toggle_gripper()
        elif self.joystick.did_button_down(("main", "A")):
            self.home_linear()
        elif self.joystick.did_button_down(("main", "X")):
            self.toggle_tilter()
        elif self.joystick.did_button_down(("main", "Y")):
            self.set_motors_active(not self.motors_active)
        
        if self.joystick.is_button_down(("triggers", "L1")):
            self.tilt_speed = 0
            self.enable_tilt_axis = True
        else:
            self.set_linear(0.0)
            self.enable_tilt_axis = False

        if self.enable_tilt_axis:
            self.tilt_speed = int(self.max_tilt_speed * self.joystick.get_axis(self.tilter_axis))

            if self.tilt_speed != 0:
                self.set_tilter(self.tilt_position + self.tilt_speed)
        else:
            stepper_val = self.joystick.deadband_axis(self.stepper_axis, self.deadzone_joy_val, 1.0)
            self.set_linear(stepper_val)

        if any(self.joystick.check_list(self.joystick.did_axis_change, self.linear_axis, self.angular_axis)):
            self.cmd_vel_timer = rospy.Time.now()
            linear_val = self.joystick.deadband_axis(self.linear_axis, self.deadzone_joy_val, self.linear_scale)
            angular_val = self.joystick.deadband_axis(self.angular_axis, self.deadzone_joy_val, self.angular_scale)

            self.set_twist(linear_val, angular_val)
        
        if self.joystick.did_axis_change(("dpad", "vertical")):
            if self.joystick.get_axis(("dpad", "vertical")) > 0.0:
                self.play_random_audio(self.soundboard_up)
            elif self.joystick.get_axis(("dpad", "vertical")) < 0.0:
                self.play_random_audio(self.soundboard_down)
        if self.joystick.did_axis_change(("dpad", "horizontal")):
            if self.joystick.get_axis(("dpad", "horizontal")) > 0.0:
                self.select_led_pattern(self.led_pattern_index - 1)
            elif self.joystick.get_axis(("dpad", "horizontal")) < 0.0:
                self.select_led_pattern(self.led_pattern_index + 1)

    def set_twist(self, linear_val, angular_val):
        if (self.twist_command.linear.x != linear_val or 
                self.twist_command.angular.z != angular_val):
            self.twist_command.linear.x = linear_val
            self.twist_command.angular.z = angular_val
    
    def set_twist_zero(self):
        self.twist_command.linear.x = 0.0
        self.twist_command.angular.z = 0.0

    def run(self):
        clock_rate = rospy.Rate(20.0)
        while not rospy.is_shutdown():
            dt = rospy.Time.now() - self.cmd_vel_timer
            if dt > self.cmd_vel_timeout:
                self.set_twist_zero()
            if dt < self.send_timeout:
                self.cmd_vel_pub.publish(self.twist_command)
            clock_rate.sleep()


if __name__ == "__main__":
    try:
        node = DodobotDebugJoystick()
        node.run()
    except rospy.ROSInterruptException:
        pass
    rospy.loginfo("Exiting db_debug_joystick node")
