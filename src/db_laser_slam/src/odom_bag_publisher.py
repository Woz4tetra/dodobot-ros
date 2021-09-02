#!/usr/bin/python3
import rospy

import tf2_ros
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped


class OdomBagPublisher(object):
    def __init__(self):
        self.node_name = "odom_bag_publisher"
        rospy.init_node(
            self.node_name
            # disable_signals=True
            # log_level=rospy.DEBUG
        )

        self.odom_sub = rospy.Subscriber("odom", Odometry, self.odom_callback, queue_size=50)

        self.br = tf2_ros.TransformBroadcaster()
        self.tf_msg = TransformStamped()

        rospy.loginfo("%s init complete" % self.node_name)

    def run(self):
        rospy.spin()

    def odom_callback(self, msg):
        self.tf_msg.header.frame_id = msg.header.frame_id
        self.tf_msg.child_frame_id = msg.child_frame_id
        self.tf_msg.header.stamp = msg.header.stamp
        self.tf_msg.transform.translation.x = msg.pose.pose.position.x
        self.tf_msg.transform.translation.y = msg.pose.pose.position.y
        self.tf_msg.transform.translation.z = msg.pose.pose.position.z
        self.tf_msg.transform.rotation.x = msg.pose.pose.orientation.x
        self.tf_msg.transform.rotation.y = msg.pose.pose.orientation.y
        self.tf_msg.transform.rotation.z = msg.pose.pose.orientation.z
        self.tf_msg.transform.rotation.w = msg.pose.pose.orientation.w

        self.br.sendTransform(self.tf_msg)


if __name__ == "__main__":
    node = OdomBagPublisher()
    try:
        node.run()

    except rospy.ROSInterruptException:
        pass

    finally:
        rospy.loginfo("Exiting %s node" % node.node_name)
