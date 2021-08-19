#!/usr/bin/env python3
import rospy

import cv2
import math
import datetime
import numpy as np

from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo

import ctypes
# a thread gets killed improperly within CvBridge without this causing segfaults
libgcc_s = ctypes.CDLL('libgcc_s.so.1')

from cv_bridge import CvBridge, CvBridgeError


class ImageHelper(object):
    def __init__(self):
        self.node_name = "image_helper"
        rospy.init_node(
            self.node_name
            # disable_signals=True
            # log_level=rospy.DEBUG
        )

        self.max_distance = rospy.get_param("~max_distance", 5.0)
        self.resize_width = rospy.get_param("~resize_width", None)
        self.resize_height = rospy.get_param("~resize_height", None)

        self.max_distance_mm = int(self.max_distance * 1000.0)

        self.bridge = CvBridge()

        self.depth_color_image_pub = rospy.Publisher("depth/image_color_raw", Image, queue_size=1)
        rospy.Subscriber("depth/image_raw", Image, self.depth_callback, queue_size=1)
        
        self.thumb_color_image_pub = rospy.Publisher("color/image_thumb_raw", Image, queue_size=1)
        rospy.Subscriber("color/image_raw", Image, self.color_callback, queue_size=1)

        rospy.loginfo("%s init complete" % self.node_name)

    def run(self):
        rospy.spin()

    def publish_image(self, publisher, image):
        try:
            image_msg = self.bridge.cv2_to_imgmsg(image, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
        else:
            publisher.publish(image_msg)

    def color_callback(self, msg):
        try:
            color_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
            return
        color_image = self.resize(color_image)
        self.publish_image(self.thumb_color_image_pub, color_image)

    def depth_callback(self, msg):
        try:
            depth_image = self.bridge.imgmsg_to_cv2(msg, "passthrough")
        except CvBridgeError as e:
            rospy.logerr(e)
            return
        
        threshold, depth_bounded = cv2.threshold(depth_image, self.max_distance_mm, 65535, cv2.THRESH_TOZERO_INV)
        normalized = np.uint8(depth_bounded.astype(np.float64) * 255 / self.max_distance_mm)
        # normalized = np.uint8(depth_bounded.astype(np.float64) * 255 / np.max(depth_bounded))
        # color_image = cv2.cvtColor(normalized, cv2.COLOR_GRAY2BGR)
        color_image = cv2.applyColorMap(normalized, cv2.COLORMAP_JET)
        color_image = self.resize(color_image)
        self.publish_image(self.depth_color_image_pub, color_image)

    def resize(self, image):
        if self.resize_width is not None or self.resize_height is not None:
            if self.resize_width is None:
                self.resize_width = image.shape[1]
            if self.resize_height is None:
                self.resize_height = image.shape[0]
            image = cv2.resize(image, (self.resize_width, self.resize_height))
        return image

if __name__ == "__main__":
    node = ImageHelper()
    try:
        node.run()

    except rospy.ROSInterruptException:
        pass

    finally:
        rospy.loginfo("Exiting %s node" % node.node_name)
