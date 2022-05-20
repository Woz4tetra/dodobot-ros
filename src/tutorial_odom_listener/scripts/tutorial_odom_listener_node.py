#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry


class TutorialOdomListener:
    def __init__(self):
        rospy.init_node(
            "tutorial_odom_listener"
        )
        self.odom_sub = rospy.Subscriber("/dodobot/odom", Odometry, self.odom_callback, queue_size=10)

    def odom_callback(self, msg):
        print("X: %0.4f, Y: %0.4f" % (msg.pose.pose.position.x, msg.pose.pose.position.y))

    def run(self):
        rospy.spin()


if __name__ == "__main__":
    try:
        node = TutorialOdomListener()
        node.run()
    except rospy.ROSInterruptException:
        pass
    rospy.loginfo("Exiting db_debug_joystick node")
