#!/usr/bin/python
from __future__ import print_function
from __future__ import division

import tf
import rospy
import numpy as np

from apriltag_ros.msg import AprilTagDetectionArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

from std_srvs.srv import Trigger, TriggerResponse


class TagConversion:
    """
    Class definition for tag_conversion ROS node.
    """

    def __init__(self):
        """
        Initializes ROS and necessary services and publishers.
        """

        self.node_name = "tag_conversion"
        rospy.init_node(
            self.node_name
            # disable_signals=True
            # log_level=rospy.DEBUG
        )
        # rospy.on_shutdown(self.shutdown_hook)

        self.services_enabled = rospy.get_param("~services_enabled", True)

        self.tf_listener = tf.TransformListener()

        self.tag_sub = rospy.Subscriber("/tag_detections", AprilTagDetectionArray, self.tag_callback, queue_size=5)
        self.odom_pub = rospy.Publisher("tag_odom", Odometry, queue_size=5)

        self.tag_odom_frame = "map"
        self.tag_child_frame = "base_link"
        self.tag_frame = "target"
        self.tag_rotated_frame = "target_rotated"
        self.tag_covariance = [
            1e-3, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 1e-3, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 1e-3, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 1e-3, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 1e-3, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 1e-3
        ]

        self.first_sighting = True

        self.tag_inital_trans = None
        self.tag_inital_rot = None
        self.tag_initial_matrix = None

        self.odom_reset_service_name = "tag_conversion_odom_reset"
        self.odom_reset = None

        if self.services_enabled:
            rospy.loginfo("Setting up service %s" % self.odom_reset_service_name)
            self.odom_reset = rospy.Service(self.odom_reset_service_name, Trigger, self.odom_reset_callback)
            rospy.loginfo("%s service is ready" % self.odom_reset_service_name)

    def odom_reset_callback(self, req):
        rospy.loginfo("Resetting tag conversion odom")
        result = self.reset_odom()
        if result is None:
            return TriggerResponse(True, "")
        else:
            return TriggerResponse(False, str(e))

    def reset_odom(self):
        try:
            self.tag_inital_trans, self.tag_inital_rot = self.tf_listener.lookupTransform(self.tag_odom_frame, self.tag_rotated_frame, rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            return e

        trans_m = tf.transformations.quaternion_matrix(self.tag_inital_rot)
        rot_m = tf.transformations.translation_matrix(self.tag_inital_trans)
        self.tag_initial_matrix = np.dot(trans_m, rot_m)

    def tf_to_odom_start(self, tfd_pose):
        # transform found tag position to the starting point of tag_odom_frame
        # this will make the odometry values line up with chassis odom
        trans_m = tf.transformations.translation_matrix([
            tfd_pose.pose.position.x,
            tfd_pose.pose.position.y,
            tfd_pose.pose.position.z
        ])
        rot_m = tf.transformations.quaternion_matrix([
            tfd_pose.pose.orientation.x,
            tfd_pose.pose.orientation.y,
            tfd_pose.pose.orientation.z,
            tfd_pose.pose.orientation.w
        ])
        tag_matrix = np.dot(trans_m, rot_m)  # create full matrix
        tfd_matrix = np.dot(tag_matrix, self.tag_initial_matrix)  # tf tag pose relative to tag_odom_frame start

        tfd_trans = tf.transformations.translation_from_matrix(tfd_matrix)
        tfd_quat = tf.transformations.quaternion_from_matrix(tfd_matrix)

        return tfd_trans, tfd_quat

    def tag_callback(self, msg):
        if len(msg.detections) == 0:
            return

        # tag_pose_with_covar_stamped = msg.detections[0].pose
        # tag_pose_stamped = PoseStamped()
        # tag_pose_stamped.header = tag_pose_with_covar_stamped.header
        # tag_pose_stamped.pose = tag_pose_with_covar_stamped.pose.pose
        #
        # try:
        #     rotated_pose = self.tf_listener.transformPose(self.tag_rotated_frame, tag_pose_stamped)
        #     print(tag_pose_stamped.pose.orientation)
        #     print(rotated_pose.pose.orientation)
        #     print()
        # except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        #     return

        if self.first_sighting:
            if self.reset_odom() is not None:
                return
            self.first_sighting = False

        tag_pose_stamped = PoseStamped()
        tag_pose_stamped.header.frame_id = self.tag_child_frame

        try:
            tfd_pose = self.tf_listener.transformPose(self.tag_rotated_frame, tag_pose_stamped)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return

        tfd_trans, tfd_quat = self.tf_to_odom_start(tfd_pose)

        odom_msg = Odometry()
        # odom_msg.pose.pose = tfd_pose.pose
        odom_msg.pose.pose.position.x = tfd_trans[0]
        odom_msg.pose.pose.position.y = tfd_trans[1]
        odom_msg.pose.pose.position.z = tfd_trans[2]
        odom_msg.pose.pose.orientation.x = tfd_quat[0]
        odom_msg.pose.pose.orientation.y = tfd_quat[1]
        odom_msg.pose.pose.orientation.z = tfd_quat[2]
        odom_msg.pose.pose.orientation.w = tfd_quat[3]
        odom_msg.pose.covariance = self.tag_covariance
        odom_msg.header.frame_id = self.tag_odom_frame
        odom_msg.header.stamp = rospy.Time.now()
        odom_msg.child_frame_id = self.tag_child_frame
        self.odom_pub.publish(odom_msg)


if __name__ == "__main__":
    try:
        node = TagConversion()
        # node.run()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass

    finally:
        rospy.loginfo("Exiting tag_conversion node")
