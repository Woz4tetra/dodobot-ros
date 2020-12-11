#!/usr/bin/python
from __future__ import print_function

import os
import cv2
import math
import rospy
import datetime
import numpy as np
import threading

import message_filters

from std_msgs.msg import ColorRGBA

from vision_msgs.msg import Detection2DArray
from vision_msgs.msg import ObjectHypothesisWithPose
from vision_msgs.msg import VisionInfo

from visualization_msgs.msg import MarkerArray
from visualization_msgs.msg import Marker

import geometry_msgs
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovariance
from geometry_msgs.msg import Vector3

from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo

from image_geometry import PinholeCameraModel


class DodobotBboxToPose:
    """
    Class definition for db_bbox_to_pose ROS node.
    """

    def __init__(self):
        """
        Initializes ROS and necessary services and publishers.
        """

        self.node_name = "db_bbox_to_pose"
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
        self.publish_debug_image = rospy.get_param("~publish_debug_image", False)
        self.publish_markers = rospy.get_param("~publish_markers", True)
        self.bounding_box_border = rospy.get_param("~bounding_box_border_px", 30)
        marker_persistance_s = rospy.get_param("~marker_persistance_s", 0.5)
        self.marker_persistance = rospy.Duration(marker_persistance_s)

        label_list = rospy.get_param("~labels", None)
        self.labels = {index: label for index, label in enumerate(label_list)}

        self.label_colors = rospy.get_param("~marker_colors", None)
        if self.publish_markers:
            assert self.label_colors is not None
            for label in self.labels.values():
                assert label in self.label_colors, label

        self.z_depth_estimations = rospy.get_param("~z_depth_estimations")
        for label in self.labels.values():
            assert label in self.z_depth_estimations, label

        self.parent_tf_name = ""
        self.camera_size = [0, 0]
        self.depth_size = [0, 0]
        self.camera_model = PinholeCameraModel()
        self.camera_info_loaded = False
        self.depth_info_loaded = False
        self.debug_image = None

        self.depth_msg = None
        self.detect_msg = None

        self.camera_info_sub = rospy.Subscriber(self.camera_info_topic, CameraInfo, self.camera_info_callback, queue_size=10)
        self.depth_info_sub = rospy.Subscriber(self.depth_info_topic, CameraInfo, self.depth_info_callback, queue_size=10)

        self.depth_sub = message_filters.Subscriber(self.depth_topic, Image, queue_size=1)
        self.detect_sub = message_filters.Subscriber(self.detect_topic, Detection2DArray, queue_size=20)
        self.approx_time_sync = message_filters.ApproximateTimeSynchronizer([self.depth_sub, self.detect_sub], queue_size=20, slop=0.01)
        self.approx_time_sync.registerCallback(self.depth_detect_callback)
        # self.depth_sub = rospy.Subscriber(self.depth_topic, Image, self.depth_callback, queue_size=25)
        # self.detect_sub = rospy.Subscriber(self.detect_topic, Detection2DArray, self.detect_callback, queue_size=25)

        self.object_poses_pub = rospy.Publisher("object_poses", Detection2DArray, queue_size=25)
        self.object_markers_pub = rospy.Publisher("object_markers", MarkerArray, queue_size=25)
        if self.publish_debug_image:
            self.debug_image_pub = rospy.Publisher("object_pose_debug_image", Image, queue_size=1)

    def camera_info_callback(self, msg):
        self.parent_tf_name = msg.header.frame_id
        self.camera_size[0] = msg.width
        self.camera_size[1] = msg.height
        self.camera_model.fromCameraInfo(msg)
        self.camera_info_loaded = True

    def depth_info_callback(self, msg):
        self.depth_size[0] = msg.width
        self.depth_size[1] = msg.height
        self.depth_info_loaded = True

    def depth_callback(self, msg):
        self.depth_msg = msg
        rospy.loginfo("depth_msg: %s" % msg.header.stamp)

    def detect_callback(self, msg):
        self.detect_msg = msg
        rospy.loginfo("detect_msg: %s" % msg.header.stamp)

    def depth_detect_callback(self, depth_msg, detect_msg):
        if not (self.camera_info_loaded and self.depth_info_loaded):
            rospy.logwarn("Camera model not loaded. Skipping detection.")
        if depth_msg is None:
            return
        if detect_msg is None:
            return

        # dt = depth_msg.header.stamp - detect_msg.header.stamp
        # rospy.loginfo("Time diff: %s" % (dt.to_sec()))

        detect_with_pose_msg = Detection2DArray()
        detect_with_pose_msg.header.frame_id = self.parent_tf_name
        detect_with_pose_msg.header.stamp = depth_msg.header.stamp

        markers = MarkerArray()

        try:
            cv_image = self.bridge.imgmsg_to_cv2(depth_msg, "passthrough")
        except CvBridgeError as e:
            rospy.logerr(e)
            return

        if self.camera_size != self.depth_size:
            cv_image = cv2.resize(cv_image, tuple(self.camera_size))
            # rospy.logwarn("Color and depth images are not the same size! %s != %s" % (self.camera_size, self.depth_size))
            # return

        if self.publish_debug_image:
            self.debug_image = np.zeros((self.camera_size[1], self.camera_size[0], 3), np.uint8)

        found_labels = {}
        for detection in detect_msg.detections:
            label_id = detection.results[0].id
            label_name = self.labels[label_id]
            if label_name not in found_labels:
                found_labels[label_name] = 0
            else:
                found_labels[label_name] += 1
            label_index = found_labels[label_name]

            pose, scale_vector = self.bbox_to_pose(detection.bbox, cv_image, label_name, depth_msg.header.stamp)
            pose_with_covar = PoseWithCovariance()
            pose_with_covar.pose = pose.pose

            detection.results[0].pose = pose_with_covar
            detect_with_pose_msg.detections.append(detection)

            sphere_marker = self.make_marker(pose, label_name, label_index, scale_vector)
            text_marker = self.make_marker(pose, label_name, label_index, Vector3(0.0, 0.0, scale_vector.z))

            sphere_marker.type = Marker.SPHERE
            sphere_marker.ns = "sphere_" + sphere_marker.ns

            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.ns = "text_" + text_marker.ns
            text_marker.text = "%s_%s" % (label_name, label_index)

            markers.markers.append(sphere_marker)
            markers.markers.append(text_marker)

        self.object_poses_pub.publish(detect_with_pose_msg)
        self.object_markers_pub.publish(markers)

        if self.publish_debug_image:
            try:
                self.debug_image_pub.publish(self.bridge.cv2_to_imgmsg(self.debug_image, "bgr8"))
            except CvBridgeError as e:
                rospy.logerr(e)

    def bbox_to_pose(self, bbox, depth_image, label, stamp):
        x_center = int(bbox.center.x)
        y_center = int(bbox.center.y)

        obj_radius = min(bbox.size_x, bbox.size_y) / 2.0
        detect_radius = max(obj_radius - self.bounding_box_border, 1.0)
        detect_radius = int(detect_radius)

        z_dist = self.get_z_dist(depth_image, x_center, y_center, detect_radius, label, self.publish_debug_image)
        z_model = self.z_depth_estimations[label]
        z_dist += z_model / 2.0

        ray = self.camera_model.projectPixelTo3dRay((x_center, y_center))
        x_dist = ray[0] * z_dist
        y_dist = ray[1] * z_dist

        # calculate rough object size for marker visualization
        x_obj = x_center - int(bbox.size_x / 2.0)
        y_obj = y_center - int(bbox.size_y / 2.0)
        ray = self.camera_model.projectPixelTo3dRay((x_obj, y_obj))
        x_obj_dist = ray[0] * z_dist
        y_obj_dist = ray[1] * z_dist
        scale_vector = Vector3()
        scale_vector.x = abs(x_obj_dist - x_dist) * 2.0
        scale_vector.y = abs(y_obj_dist - y_dist) * 2.0
        scale_vector.z = z_model  # no measurement for Z depth, use external input

        rospy.logdebug("label: %s, X: %0.3f, Y: %0.3f, Z: %0.3f" % (label,
            x_dist, y_dist, z_dist)
        )

        pose = PoseStamped()
        pose.header.frame_id = self.camera_model.tfFrame()
        # pose.header.stamp = rospy.Time.now()
        pose.header.stamp = stamp
        pose.pose.position.x = x_dist
        pose.pose.position.y = y_dist
        pose.pose.position.z = z_dist
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = 0.0
        pose.pose.orientation.w = 1.0

        return pose, scale_vector

    def get_z_dist(self, depth_image, x_center, y_center, detect_radius, label, draw_debug=True):
        depth_array = np.array(depth_image, dtype=np.float32)
        circle_mask = np.zeros((depth_array.shape[0], depth_array.shape[1]), np.uint8)
        cv2.circle(circle_mask, (x_center, y_center), detect_radius, (255,255,255),-1)
        nonzero_mask = np.array(depth_image > 0.0, np.uint8)
        z_dist = cv2.mean(depth_image, mask=cv2.bitwise_and(circle_mask, nonzero_mask))[0]  # depth values are in mm
        z_dist /= 1000.0
        if draw_debug:
            masked = cv2.bitwise_and(depth_image, depth_image, mask=circle_mask)
            masked_color = np.array(cv2.cvtColor(masked, cv2.COLOR_GRAY2BGR), np.uint8)
            # masked_color = cv2.resize(masked_color, self.camera_size)

            cv2.circle(masked_color, (x_center, y_center), detect_radius, (0, 0, 255), 2)
            cv2.putText(masked_color, label, (x_center - detect_radius, y_center), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 200), 2, cv2.LINE_AA)
            self.debug_image = cv2.bitwise_or(self.debug_image, masked_color)
        return z_dist

    def make_marker(self, pose_stamped, label, index, scale_vector):
        marker = Marker()
        marker.action = Marker.ADD
        marker.pose = pose_stamped.pose
        marker.header = pose_stamped.header
        marker.lifetime = self.marker_persistance
        marker.ns = label
        marker.id = index
        marker.scale = scale_vector

        rgba = ColorRGBA()
        color_array = self.label_colors[label]
        rgba.r = color_array[0]
        rgba.g = color_array[1]
        rgba.b = color_array[2]
        rgba.a = color_array[3]

        marker.color = rgba

        return marker

    def run(self):
        # rate = rospy.Rate(10.0)
        # while not rospy.is_shutdown():
        #     self.depth_detect_callback(self.depth_msg, self.detect_msg)
        #     rate.sleep()

        rospy.spin()

    def shutdown_hook(self):
        pass

if __name__ == "__main__":
    try:
        node = DodobotBboxToPose()
        node.run()

    except rospy.ROSInterruptException:
        pass

    finally:
        rospy.loginfo("Exiting db_bbox_to_pose node")
