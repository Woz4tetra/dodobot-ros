#!/usr/bin/python
from __future__ import print_function

import os
import cv2
import math
import rospy
import datetime
import numpy as np

import message_filters

import tf2_ros
import tf_conversions
import tf2_geometry_msgs

from vision_msgs.msg import Detection2DArray
from vision_msgs.msg import ObjectHypothesisWithPose
from vision_msgs.msg import VisionInfo

import geometry_msgs
from geometry_msgs.msg import PoseWithCovarianceStamped

from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo

from image_geometry import PinholeCameraModel

from stored_object import StoredObjectContainer, StoredObject


class DodobotObjectTracking:
    """
    Class definition for db_object_tracking ROS node.
    """

    def __init__(self):
        """
        Initializes ROS and necessary services and publishers.
        """

        self.node_name = "db_object_tracking"
        rospy.init_node(
            self.node_name
            # disable_signals=True
            # log_level=rospy.DEBUG
        )
        # rospy.on_shutdown(self.shutdown_hook)

        self.bridge = CvBridge()

        self.depth_topic = rospy.get_param("~depth_topic", "depth/image_raw")
        self.camera_info_topic = rospy.get_param("~camera_info_topic", "color/camera_info")
        self.depth_info_topic = rospy.get_param("~depth_info_topic", "depth/camera_info")
        self.detect_topic = rospy.get_param("~detect_topic", "/detectnet/detections")
        self.publish_label_tfs = rospy.get_param("~publish_label_tfs", True)
        self.publish_debug_image = rospy.get_param("~publish_debug_image", True)
        self.bounding_box_border = rospy.get_param("~bounding_box_border_px", 30)
        self.main_workspace_tf = rospy.get_param("~map_tf_name", "map")
        self.close_obj_threshold_m = rospy.get_param("~close_obj_threshold_m", 0.1)

        label_list = rospy.get_param("~labels", None)  # tag's TF name
        self.labels = {index: label for index, label in enumerate(label_list)}

        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.parent_tf_name = ""
        self.camera_size = [0, 0]
        self.depth_size = [0, 0]
        self.camera_model = PinholeCameraModel()
        self.camera_info_loaded = False
        self.depth_info_loaded = False
        self.debug_image = None

        self.stored_objects = StoredObjectContainer(self.camera_model, self.close_obj_threshold_m)

        self.camera_info_sub = rospy.Subscriber(self.camera_info_topic, CameraInfo, self.camera_info_callback, queue_size=10)
        self.depth_info_sub = rospy.Subscriber(self.depth_info_topic, CameraInfo, self.depth_info_callback, queue_size=10)

        self.depth_sub = message_filters.Subscriber(self.depth_topic, Image, queue_size=5)
        self.detect_sub = message_filters.Subscriber(self.detect_topic, Detection2DArray, queue_size=10)
        self.approx_time_sync = message_filters.ApproximateTimeSynchronizer([self.depth_sub, self.detect_sub], queue_size=10, slop=0.02)
        self.approx_time_sync.registerCallback(self.depth_detect_callback)

        self.object_poses_pub = rospy.Publisher("object_poses", Detection2DArray, queue_size=25)
        self.debug_image_pub = rospy.Publisher("object_pose_debug_image", Image, queue_size=1)

    def camera_info_callback(self, msg):
        self.parent_tf_name = msg.header.frame_id
        self.camera_size[0] = msg.width
        self.camera_size[1] = msg.height
        self.camera_model.fromCameraInfo(msg)
        # rospy.loginfo("Camera model loaded!")
        self.camera_info_loaded = True

    def depth_info_callback(self, msg):
        self.depth_size[0] = msg.width
        self.depth_size[1] = msg.height
        # rospy.loginfo("Depth size loaded!")
        self.depth_info_loaded = True

    def depth_detect_callback(self, depth_msg, detect_msg):
        if not (self.camera_info_loaded and self.depth_info_loaded):
            rospy.logwarn("Camera model not loaded. Skipping detection.")
        detect_with_pose_msg = Detection2DArray()

        try:
            cv_image = self.bridge.imgmsg_to_cv2(depth_msg, "passthrough")
        except CvBridgeError as e:
            rospy.logerr(e)
            return

        if self.camera_size != self.depth_size:
            rospy.logwarn("Color and depth images are not the same size! %s != %s" % (self.camera_size, self.depth_size))
            return

        if self.publish_debug_image:
            self.debug_image = np.zeros((self.camera_size[1], self.camera_size[0], 3), np.uint8)

        try:
            camera_tf = self.tf_buffer.lookup_transform(
                self.main_workspace_tf,
                self.camera_model.tfFrame(), detect_msg.header.stamp, # - rospy.Duration(0.25),
                rospy.Duration(0.1)
            )
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logerr_throttle(15.0, e)
            camera_tf = None

        # label_counters = {}
        in_view_objects = []
        for detection in detect_msg.detections:
            label_id = detection.results[0].id
            label_name = self.labels[label_id]
            # if label_id not in label_counters:
            #     label_counters[label_id] = 0
            # else:
            #     label_counters[label_id] += 1
            # label_index = label_counters[label_id]
            #
            # child_frame = "%s_%s" % (label_name, label_index)

            pose = self.bbox_to_pose(detection.bbox, cv_image, label_name)
            if pose is None:
                continue

            detection.results[0].pose = pose
            detect_with_pose_msg.detections.append(detection)
            # self.publish_detected_tf(child_frame, self.parent_tf_name, pose.pose.pose)

            if camera_tf is not None:
                # transform object pose into the map frame so positions stay constant while moving
                tfd_pose = tf2_geometry_msgs.do_transform_pose(pose.pose, camera_tf)

                in_view_objects.append(StoredObject(
                    label_name, tfd_pose.pose, pose.pose.pose
                ))
                rospy.loginfo("[%s] label: %s, X: %0.3f, Y: %0.3f, Z: %0.3f" % (self.main_workspace_tf, label_name,
                    tfd_pose.pose.position.x, tfd_pose.pose.position.y, tfd_pose.pose.position.z)
                )
            else:
                rospy.loginfo("[%s] label: %s, X: %0.3f, Y: %0.3f, Z: %0.3f" % (self.camera_model.tfFrame(), label_name,
                    pose.pose.pose.position.x, pose.pose.pose.position.y, pose.pose.pose.position.z)
                )

        if camera_tf is not None:
            self.stored_objects.check_objects_in_view(in_view_objects)
            self.stored_objects.check_in_view_points(in_view_objects)

        self.object_poses_pub.publish(detect_with_pose_msg)

        if self.publish_debug_image:
            try:
                self.debug_image_pub.publish(self.bridge.cv2_to_imgmsg(self.debug_image, "bgr8"))
            except CvBridgeError as e:
                rospy.logerr(e)

    def bbox_to_pose(self, bbox, depth_image, label):
        depth_array = np.array(depth_image, dtype=np.float32)

        x_center = int(bbox.center.x)
        y_center = int(bbox.center.y)

        radius = min(bbox.size_x, bbox.size_y) / 2.0
        radius = max(radius - self.bounding_box_border, 1.0)
        radius = int(radius)
        # radius = 10
        circle_mask = np.zeros((depth_array.shape[0], depth_array.shape[1]), np.uint8)
        cv2.circle(circle_mask, (x_center, y_center), radius, (255,255,255),-1)
        nonzero_mask = np.array(depth_image > 0.0, np.uint8)
        z_dist = cv2.mean(depth_image, mask=cv2.bitwise_and(circle_mask, nonzero_mask))[0]  # depth values are in mm
        z_dist /= 1000.0
        if self.publish_debug_image:
            masked = cv2.bitwise_and(depth_image, depth_image, mask=circle_mask)
            masked_color = np.array(cv2.cvtColor(masked, cv2.COLOR_GRAY2BGR), np.uint8)

            cv2.circle(masked_color, (x_center, y_center), radius, (0, 0, 255), 2)
            cv2.putText(masked_color, label, (x_center - radius, y_center), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 200), 2, cv2.LINE_AA)
            self.debug_image = cv2.bitwise_or(self.debug_image, masked_color)

        ray = self.camera_model.projectPixelTo3dRay((x_center, y_center))

        x_dist = ray[0] * z_dist
        y_dist = ray[1] * z_dist

        # rospy.loginfo("label: %s, X: %0.3f, Y: %0.3f, Z: %0.3f" % (label,
        #     x_dist, y_dist, z_dist)
        # )

        pose = PoseWithCovarianceStamped()
        pose.pose.pose.position.x = x_dist
        pose.pose.pose.position.y = y_dist
        pose.pose.pose.position.z = z_dist
        pose.pose.pose.orientation.x = 0.0
        pose.pose.pose.orientation.y = 0.0
        pose.pose.pose.orientation.z = 0.0
        pose.pose.pose.orientation.w = 1.0

        return pose

    def publish_detected_tf(self, child_frame, parent_frame, pose):
        tf_msg = geometry_msgs.msg.TransformStamped()
        tf_msg.header.stamp = rospy.Time.now()
        tf_msg.header.frame_id = parent_frame
        tf_msg.child_frame_id = child_frame
        tf_msg.transform.translation.x = pose.position.x
        tf_msg.transform.translation.y = pose.position.y
        tf_msg.transform.translation.z = pose.position.z
        tf_msg.transform.rotation.x = pose.orientation.x
        tf_msg.transform.rotation.y = pose.orientation.y
        tf_msg.transform.rotation.z = pose.orientation.z
        tf_msg.transform.rotation.w = pose.orientation.w

        self.tf_broadcaster.sendTransform(tf_msg)

    def run(self):
        # camera_info_msg = rospy.wait_for_message(self.camera_info_topic, CameraInfo, timeout=30.0)
        # depth_info_msg = rospy.wait_for_message(self.depth_info_topic, CameraInfo, timeout=30.0)
        # self.camera_info_callback(camera_info_msg)
        # self.depth_info_callback(depth_info_msg)
        # self.camera_info_loaded = True
        # rospy.spin()

        rate = rospy.Rate(5.0)
        while not rospy.is_shutdown():
            active_tf_names = []
            for child_frame, pose in self.stored_objects.iter_tfs():
                self.publish_detected_tf(child_frame, self.main_workspace_tf, pose)
                active_tf_names.append(child_frame)
            rospy.loginfo_throttle(1.0, "Stored objects: %s" % str(active_tf_names))
            rate.sleep()

    def shutdown_hook(self):
        pass

if __name__ == "__main__":
    try:
        node = DodobotObjectTracking()
        node.run()

    except rospy.ROSInterruptException:
        pass

    finally:
        rospy.loginfo("Exiting db_object_tracking node")