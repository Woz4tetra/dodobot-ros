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
        self.color_resize_width = rospy.get_param("~color_resize_width", None)
        self.color_resize_height = rospy.get_param("~color_resize_height", None)
        self.depth_resize_width = rospy.get_param("~depth_resize_width", None)
        self.depth_resize_height = rospy.get_param("~depth_resize_height", None)
        self.frame_skip = rospy.get_param("~frame_skip", 0)

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
        if self.frame_skip > 1 and msg.header.seq % self.frame_skip != 0:
            return
        try:
            color_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
            return
        color_image = self.resize(color_image, self.color_resize_width, self.color_resize_height)
        self.publish_image(self.thumb_color_image_pub, color_image)

    def depth_callback(self, msg):
        if self.frame_skip > 1 and msg.header.seq % self.frame_skip != 0:
            return
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
        color_image = self.resize(color_image, self.depth_resize_width, self.depth_resize_height)
        self.publish_image(self.depth_color_image_pub, color_image)

    def resize(self, image, resize_width, resize_height):
        if resize_width is not None or resize_height is not None:
            if resize_width is None:
                resize_width = image.shape[1]
            if resize_height is None:
                resize_height = image.shape[0]
            image = cv2.resize(image, (resize_width, resize_height))
        return image

if __name__ == "__main__":
    node = ImageHelper()
    try:
        node.run()

    except rospy.ROSInterruptException:
        pass

    finally:
        rospy.loginfo("Exiting %s node" % node.node_name)
