#!/usr/bin/python
from __future__ import print_function
from __future__ import division

import tf
import rospy
import numpy as np

from apriltag_ros.msg import AprilTagDetectionArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped

from std_srvs.srv import Trigger, TriggerResponse
from std_srvs.srv import SetBool, SetBoolResponse
from robot_localization.srv import SetPose, SetPoseResponse

from db_chassis.srv import DodobotOdomReset, DodobotOdomResetResponse


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

        self.tf_listener = tf.TransformListener(cache_time=rospy.Duration(120.0))
        self.tf_broadcaster = tf.TransformBroadcaster()

        self.tag_odom_frame = "odom"
        self.publish_odom_frame = "odom"
        self.publish_map_frame = "map"
        self.tag_child_frame = "base_link"
        self.base_link_frame = "base_link"
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
        self.tag_is_visible = False
        # self.is_active = False
        self.is_active = True

        self.active_clock_rate = rospy.Rate(30)
        self.inactive_clock_rate = rospy.Rate(1)
        self.clock_rate = self.inactive_clock_rate

        self.tag_initial_trans = None
        self.tag_initial_rot = None
        self.tag_initial_matrix = None

        # self.odom_current_trans = None
        # self.odom_current_rot = None

        self.odom_initial_trans = None
        self.odom_initial_rot = None
        self.odom_initial_matrix = None

        self.tag_odom_trans = [0.0, 0.0, 0.0]
        self.tag_odom_quat = [0.0, 0.0, 0.0, 1.0]

        self.odom_reset_service_name = "tag_conversion_odom_reset"
        self.odom_reset_srv = None

        self.active_service_name = "tag_conversion_active"
        self.active_srv = None

        self.ekf_reset_service_name = "/set_pose"
        self.ekf_reset_srv = None

        self.chassis_odom_reset_service_name = "dodobot_odom_reset"
        self.chassis_odom_reset_srv = None

        if self.is_active:
            self.subscribe_to_topics()

        if self.services_enabled:
            rospy.loginfo("Setting up service %s" % self.odom_reset_service_name)
            self.odom_reset_srv = rospy.Service(self.odom_reset_service_name, Trigger, self.odom_reset_callback)
            rospy.loginfo("%s service is ready" % self.odom_reset_service_name)

            rospy.loginfo("Setting up service %s" % self.active_service_name)
            self.active_srv = rospy.Service(self.active_service_name, SetBool, self.active_callback)
            rospy.loginfo("%s service is ready" % self.active_service_name)

            rospy.loginfo("Waiting for service %s" % self.ekf_reset_service_name)
            self.ekf_reset_srv = rospy.ServiceProxy(self.ekf_reset_service_name, SetPose)
            rospy.loginfo("%s service is ready" % self.ekf_reset_service_name)

            rospy.loginfo("Waiting for service %s" % self.chassis_odom_reset_service_name)
            self.chassis_odom_reset_srv = rospy.ServiceProxy(self.chassis_odom_reset_service_name, DodobotOdomReset)
            rospy.loginfo("%s service is ready" % self.chassis_odom_reset_service_name)

        rospy.loginfo("%s is ready" % self.node_name)

    def subscribe_to_topics(self):
        rospy.loginfo("%s is subscribing to topics" % self.node_name)
        self.tag_sub = rospy.Subscriber("/tag_detections", AprilTagDetectionArray, self.tag_callback, queue_size=5)
        self.ekf_sub = rospy.Subscriber("odom/filtered", Odometry, self.filtered_odom_callback, queue_size=5)
        # self.chassis_odom_sub = rospy.Subscriber("odom", Odometry, self.chassis_odom_callback, queue_size=5)
        self.odom_pub = rospy.Publisher("tag_odom", Odometry, queue_size=5)

    def unsubscribe_from_topics(self):
        rospy.loginfo("%s is pausing topic subscriptions" % self.node_name)
        del self.tag_sub
        del self.ekf_sub
        # del self.chassis_odom_sub
        del self.odom_pub

    def odom_reset_callback(self, req):
        if self.reset_odom():
            self.reset_ekf()
            return TriggerResponse(True, "")
        else:
            return TriggerResponse(False, str(result))

    def reset_odom(self):
        rospy.loginfo("Resetting tag conversion odom")

        result = self.chassis_odom_reset_srv(0.0, 0.0, 0.0)
        if not result.resp:
            rospy.logwarn("Failed to reset chassis odom. Skipping tag reset")
            return False

        try:
            # base_link and odom should be on top of each other now
            self.tag_initial_trans, self.tag_initial_rot = self.tf_listener.lookupTransform(self.base_link_frame, self.tag_rotated_frame, rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            return False

        # if self.odom_current_trans is None or self.odom_current_rot is None:
        #     return False

        # Flipped TF order to cancel out odom/filtered values when computing
        # from odom TF
        # self.odom_initial_trans = [-value for value in self.odom_current_trans]
        # odom_current_euler = tf.transformations.euler_from_quaternion(self.odom_current_rot)
        # euler_flipped = [-value for value in odom_current_euler]
        # self.odom_initial_rot = tf.transformations.quaternion_from_euler(euler_flipped)

        trans_m = tf.transformations.quaternion_matrix(self.tag_initial_rot)
        rot_m = tf.transformations.translation_matrix(self.tag_initial_trans)
        self.tag_initial_matrix = np.dot(trans_m, rot_m)

        self.odom_initial_rot = [0.0, 0.0, 0.0, 1.0]
        self.odom_initial_trans = [0.0, 0.0, 0.0]
        trans_m = tf.transformations.quaternion_matrix(self.odom_initial_rot)
        rot_m = tf.transformations.translation_matrix(self.odom_initial_trans)
        self.odom_initial_matrix = np.dot(trans_m, rot_m)
        rospy.loginfo("Tag conversion odom reset successful")

        return True

    def reset_ekf(self):
        if not self.services_enabled:
            rospy.logwarn("Services for this node aren't enabled! Failed to call %s" % self.ekf_reset_service_name)
            return

        if self.ekf_reset_srv is None:
            rospy.logwarn("%s service not ready yet!" % self.ekf_reset_service_name)
            return

        reset_pose = PoseWithCovarianceStamped()
        reset_pose.header.frame_id = self.tag_odom_frame
        reset_pose.pose.covariance = self.tag_covariance
        self.assign_to_pose(reset_pose.pose.pose, self.odom_initial_trans, self.odom_initial_rot)

        rospy.loginfo("Setting EKF pose to %s" % reset_pose.pose.pose)
        self.ekf_reset_srv(reset_pose)
        rospy.loginfo("Set EKF pose successful!")

    def active_callback(self, req):
        self.is_active = req.data
        if self.is_active:
            self.first_sighting = True
            self.clock_rate = self.active_clock_rate
            self.subscribe_to_topics()
        else:
            self.clock_rate = self.inactive_clock_rate
            self.unsubscribe_to_topics()

        return SetBoolResponse(True, "")

    def tf_to_start(self, pose, initial_matrix):
        trans, quat = self.assign_to_lists(pose)
        trans_m = tf.transformations.translation_matrix(trans)
        rot_m = tf.transformations.quaternion_matrix(quat)
        tag_matrix = np.dot(trans_m, rot_m)  # create full matrix
        tfd_matrix = np.dot(tag_matrix, initial_matrix)  # tf tag pose relative to tag_odom_frame start

        tfd_trans = tf.transformations.translation_from_matrix(tfd_matrix)
        tfd_quat = tf.transformations.quaternion_from_matrix(tfd_matrix)

        return tfd_trans, tfd_quat

    def tag_callback(self, msg):
        self.tag_is_visible = len(msg.detections) != 0

    def run(self):
        while not rospy.is_shutdown():
            self.publish_odom()
            self.clock_rate.sleep()

    def assign_to_pose(self, pose, trans_xyz, quat_xyzw):
        pose.position.x = trans_xyz[0]
        pose.position.y = trans_xyz[1]
        pose.position.z = trans_xyz[2]
        pose.orientation.x = quat_xyzw[0]
        pose.orientation.y = quat_xyzw[1]
        pose.orientation.z = quat_xyzw[2]
        pose.orientation.w = quat_xyzw[3]

    def assign_to_lists(self, pose):
        trans_xyz = [
            pose.position.x,
            pose.position.y,
            pose.position.z,
        ]
        quat_xyzw = [
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        ]
        return trans_xyz, quat_xyzw

    # def chassis_odom_callback(self, msg):
    #     self.odom_current_trans, self.odom_current_rot = self.assign_to_lists(msg.pose.pose)

    def publish_odom(self):
        if not self.is_active:
            return

        if not self.tag_is_visible:
            return

        if self.first_sighting:
            if not self.reset_odom():
                return
            self.reset_ekf()
            self.first_sighting = False

        tag_pose_stamped = PoseStamped()
        tag_pose_stamped.header.frame_id = self.tag_child_frame

        try:
            # tag positions will persist for as long as the cache_time parameter for tf_listener
            # So if the tag goes out of sight while doing a sequence, the map TF will still be published
            tfd_pose = self.tf_listener.transformPose(self.tag_rotated_frame, tag_pose_stamped)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return

        # transform found tag position to the starting point of tag_odom_frame
        # this will make the odometry values line up with chassis odom
        self.tag_odom_trans, self.tag_odom_quat = self.tf_to_start(tfd_pose.pose, self.tag_initial_matrix)

        now = rospy.Time.now()

        odom_msg = Odometry()
        self.assign_to_pose(odom_msg.pose.pose, self.tag_odom_trans, self.tag_odom_quat)
        odom_msg.pose.covariance = self.tag_covariance
        odom_msg.header.frame_id = self.publish_odom_frame
        odom_msg.header.stamp = now
        odom_msg.child_frame_id = self.tag_child_frame
        self.odom_pub.publish(odom_msg)

    def filtered_odom_callback(self, msg):
        if not self.is_active:
            return

        if not self.tag_is_visible:
            return

        if self.first_sighting:
            return

        if self.tag_initial_matrix is None:
            return

        # filtered_pose_stamped = PoseStamped()
        # filtered_pose_stamped.header = msg.header
        # try:
        #     tfd_pose = self.tf_listener.transformPose(self.tag_odom_frame, filtered_pose_stamped)
        # except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        #     return
        # trans, quat = self.tf_to_start(msg.pose.pose, self.odom_initial_matrix)

        trans, quat = self.assign_to_lists(msg.pose.pose)
        # trans, quat = self.assign_to_lists(tfd_pose.pose)
        now = rospy.Time.now()

        self.tf_broadcaster.sendTransform(
            trans, quat, now,
            self.base_link_frame,
            self.publish_odom_frame
        )


if __name__ == "__main__":
    try:
        node = TagConversion()
        node.run()
        # rospy.spin()

    except rospy.ROSInterruptException:
        pass

    finally:
        rospy.loginfo("Exiting tag_conversion node")
