#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from evdev import InputDevice, KeyEvent, categorize, ecodes

from key_map import *
from keyboard_listener.msg import KeyEvent

class KeyboardListener(object):
    def __init__(self):
        self.node_name = "keyboard_listener"
        rospy.init_node(
            self.node_name
            # disable_signals=True
            # log_level=rospy.DEBUG
        )

        self.key_msg = KeyEvent()
        self.key_pub = rospy.Publisher("keys", KeyEvent, queue_size=500)

        self.device_name = rospy.get_param("~device", "/dev/input/event3")
        self.device = InputDevice(self.device_name)

        self.capslock = False
        self.shift = False

    def update_caps_lock(self):
        leds = self.device.leds()
        self.capslock = ecodes.LED_CAPSL in leds

    def update_shift(self, data):
        if data.scancode in SHIFT_KEYS:
            self.shift = data.keystate == KeyEvent.key_down or data.keystate == KeyEvent.key_hold

    def run(self):
        while True:
            for event in self.device.read_loop():
                if event.type == ecodes.EV_KEY:
                    data = categorize(event)
                    # data.keycode  # string containing name of the key
                    # data.scancode  # key index
                    # data.keystate  # KeyEvent: key_down, key_up, key_hold

                    self.update_shift(data)
                    self.update_caps_lock()

                    if self.shift and data.scancode in UPPERCASE_KEYS:
                        character = UPPERCASE_KEYS[data.scancode]
                    elif self.capslock and data.scancode in CAPSLOCK_KEYS:
                        character = CAPSLOCK_KEYS[data.scancode]
                    elif data.scancode in LOWERCASE_KEYS:
                        character = LOWERCASE_KEYS[data.scancode]
                    else:
                        print "Unrecognized keyboard event: (%s (%s), %s)" % (data.keycode, data.scancode, data.keystate)
                        continue

                    self.key_msg.data = character
                    self.key_msg.event_type = data.keystate
                    self.key_pub.publish(self.key_msg)


if __name__ == "__main__":
    node = KeyboardListener()
    try:
        node.run()

    except rospy.ROSInterruptException:
        pass

    finally:
        rospy.loginfo("Exiting %s node" % node.node_name)
