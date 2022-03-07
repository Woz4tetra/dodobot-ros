#!/usr/bin/python3
import rospy
import numpy as np

from nav_msgs.msg import Odometry


class OdomListener(object):
    def __init__(self):
        self.node_name = "odom_listener"
        rospy.init_node(
            self.node_name
            # disable_signals=True
            # log_level=rospy.DEBUG
        )

        self.odom_sub = rospy.Subscriber("odom", Odometry, self.odom_callback, queue_size=50)

        self.time_data = []
        self.x_data = []
        self.y_data = []
        self.t_data = []
        self.data_length = 25

        self.x_max = 0.0
        self.y_max = 0.0
        self.t_max = 0.0

        rospy.loginfo("%s init complete" % self.node_name)

    def run(self):
        rospy.spin()

    def odom_callback(self, msg):
        self.time_data.append(msg.header.stamp.to_sec())
        self.x_data.append(msg.twist.twist.linear.x)
        self.y_data.append(msg.twist.twist.linear.y)
        self.t_data.append(msg.twist.twist.angular.z)

        self.trim_list(self.time_data, self.data_length)
        self.trim_list(self.x_data, self.data_length)
        self.trim_list(self.y_data, self.data_length)
        self.trim_list(self.t_data, self.data_length)

        if len(self.time_data) <= 1:
            return

        time_diff = np.mean(np.diff(self.time_data))
        x_accel = np.mean(np.diff(self.x_data)) / time_diff
        y_accel = np.mean(np.diff(self.y_data)) / time_diff
        t_accel = np.mean(np.diff(self.t_data)) / time_diff

        if x_accel > self.x_max:
            self.x_max = x_accel
        if y_accel > self.y_max:
            self.y_max = y_accel
        if t_accel > self.t_max:
            self.t_max = t_accel

        rospy.loginfo_throttle(1.0, "accel X: %0.5f\tY: %0.5f\tT: %0.5f" % (self.x_max, self.y_max, self.t_max))
        rospy.loginfo_throttle(1.0, "vel   X: %0.5f\tY: %0.5f\tT: %0.5f" % (np.max(np.abs(self.x_data)), np.max(np.abs(self.y_data)), np.max(np.abs(self.t_data))))
        rospy.loginfo_throttle(1.0, "delay: %0.4f" % (rospy.Time.now() - msg.header.stamp).to_sec())
    
    def trim_list(self, data, length):
        while len(data) > length:
            data.pop(0)


if __name__ == "__main__":
    node = OdomListener()
    try:
        node.run()

    except rospy.ROSInterruptException:
        pass

    finally:
        rospy.loginfo("Exiting %s node" % node.node_name)
