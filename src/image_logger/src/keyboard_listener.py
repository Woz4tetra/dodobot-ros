#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# from: https://github.com/ros-teleop/teleop_tools
# Copyright (c) 2013 PAL Robotics SL.
# Released under the BSD License.
#
# Authors:
#   * Siegfried-A. Gevatter

import curses
import math

import rospy
from std_msgs.msg import Int32


class TextWindow():
    _screen = None
    _window = None
    _num_lines = None

    def __init__(self, stdscr, lines=10):
        self._screen = stdscr
        self._screen.nodelay(True)
        curses.curs_set(0)

        self._num_lines = lines

    def read_key(self):
        keycode = self._screen.getch()
        if keycode == 3:
            raise KeyboardInterrupt
        elif keycode == 26:
            raise EOFError
        return keycode if keycode != -1 else None

    def clear(self):
        self._screen.clear()

    def write_line(self, lineno, message):
        if lineno < 0 or lineno >= self._num_lines:
            raise ValueError("lineno out of bounds")
        height, width = self._screen.getmaxyx()
        y = int((height / self._num_lines) * lineno)
        x = 10
        for text in message.split("\n"):
            text = text.ljust(width)
            self._screen.addstr(y, x, text)
            y += 1

    def refresh(self):
        self._screen.refresh()

    def beep(self):
        curses.flash()

class KeyNode():
    _interface = None

    _linear = None
    _angular = None

    def __init__(self, interface):
        self._interface = interface
        self._key_msg = Int32()
        self._pub_cmd = rospy.Publisher("keys", Int32)

        self._hz = rospy.get_param("~hz", 10)

    def run(self):
        rate = rospy.Rate(self._hz)
        while True:
            keycode = self._interface.read_key()

            if keycode and self._key_pressed(keycode):
                self._publish()

            self._update_display()
            rate.sleep()

    def _key_pressed(self, keycode):
        self._key_msg.data = keycode
        if keycode == ord("q"):
            # self._interface.beep()
            rospy.signal_shutdown("Exiting")
            raise KeyboardInterrupt

        return True

    def _update_display(self):
        self._interface.clear()
        self._interface.write_line(0, "Key: %d" % (self._key_msg.data))
        self._interface.write_line(1, "q to exit.")
        self._interface.refresh()

    def _publish(self):
        self._pub_cmd.publish(self._key_msg)


def main(stdscr):
    rospy.init_node("keyboard_listener")
    app = KeyNode(TextWindow(stdscr))
    app.run()

if __name__ == "__main__":
    try:
        curses.wrapper(main)
    except rospy.ROSInterruptException:
        rospy.loginfo("keyboard_listener interrupted by ROS")
