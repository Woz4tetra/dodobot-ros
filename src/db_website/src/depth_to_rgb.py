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


class DepthToRGB(object):
    def __init__(self):
        self.node_name = "depth_to_rgb"
        rospy.init_node(
            self.node_name
            # disable_signals=True
            # log_level=rospy.DEBUG
        )

        self.max_distance = rospy.get_param("~max_distance", 5.0)
        self.max_distance_mm = int(self.max_distance * 1000.0)

        self.bridge = CvBridge()

        self.color_image_pub = rospy.Publisher("depth/image_color_raw", Image, queue_size=1)
        rospy.Subscriber("depth/image_raw", Image, self.depth_callback, queue_size=1)
        
        rospy.loginfo("%s init complete" % self.node_name)

    def run(self):
        rospy.spin()
    

    def publish_image(self, image):
        try:
            image_msg = self.bridge.cv2_to_imgmsg(image, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
        else:
            self.color_image_pub.publish(image_msg)

    def depth_callback(self, msg):
        try:
            cv2_img = self.bridge.imgmsg_to_cv2(msg, "passthrough")
        except CvBridgeError as e:
            rospy.logerr(e)
            return
        if self.color_image_pub.get_num_connections() == 0:
            continue
        normalized = np.uint8(depth_bounded.astype(np.float64) * 255 / self.max_distance_mm)
        color_image = cv2.cvtColor(normalized, cv2.COLOR_GRAY2BGR)
        self.publish_image(color_image)


if __name__ == "__main__":
    node = DepthToRGB()
    try:
        node.run()

    except rospy.ROSInterruptException:
        pass

    finally:
        rospy.loginfo("Exiting %s node" % node.node_name)
