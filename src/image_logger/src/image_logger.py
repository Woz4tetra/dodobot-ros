#!/usr/bin/python

import os
import cv2
import pprint

import rospy

import message_filters

from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo

from std_msgs.msg import Int32

from cv_bridge import CvBridge, CvBridgeError

class ImageLogger:
    def __init__(self):
        self.node_name = "image_logger"
        rospy.init_node(
            self.node_name
            # disable_signals=True
            # log_level=rospy.DEBUG
        )
        # rospy.on_shutdown(self.shutdown_hook)

        self.save_dir = rospy.get_param("~save_dir", ".")
        self.image_topic = rospy.get_param("~image_topic", "/camera/color/image_raw")
        # self.depth_topic = rospy.get_param("~depth_topic", "/camera/depth/image_raw")
        # self.camera_info_topic = rospy.get_param("~camera_info_topic", "/camera/color/camera_info")
        self.keyboard_topic = rospy.get_param("~keyboard_topic", "keys")

        self.bridge = CvBridge()

        # self.image_sub = message_filters.Subscriber(self.image_topic, Image, queue_size=25)
        # self.depth_sub = message_filters.Subscriber(self.depth_topic, Image, queue_size=25)
        # self.camera_info_sub = message_filters.Subscriber(self.camera_info_topic, CameraInfo, queue_size=25)
        # self.approx_time_sync = message_filters.ApproximateTimeSynchronizer([self.image_sub, self.depth_sub, self.camera_info_sub], queue_size=25, slop=0.005)
        # self.approx_time_sync.registerCallback(self.camera_callback)
        rospy.Subscriber(self.image_topic, Image, self.image_callback)

        rospy.Subscriber(self.keyboard_topic, Int32, self.keyboard_callback)

        self.capture_images = False
        self.labels = {
            "0": "BACKGROUND",
            "1": "cosmo_cube",
            "2": "blue_cut_sphere",
            "3": "red_cut_sphere",
            "4": "blue_low_bin",
            "5": "red_low_bin",
            "6": "blue_cube",
            "7": "red_cube",

            "8": "blue_low_bin_and_cosmo_cube",
            "9": "blue_low_bin_and_blue_cut_sphere",
            "w": "blue_low_bin_and_red_cut_sphere",
            "e": "blue_low_bin_and_blue_cube",
            "r": "blue_low_bin_and_red_cube",

            "t": "red_low_bin_and_cosmo_cube",
            "y": "red_low_bin_and_blue_cut_sphere",
            "u": "red_low_bin_and_red_cut_sphere",
            "i": "red_low_bin_and_blue_cube",
            "o": "red_low_bin_and_red_cube",

            "p": "assorted"
        }
        self.image_label_key = list(self.labels.keys())[0]
        pprint.pprint(self.labels)
        print("Press space in the keyboard node to start recording")
        print("Currently selected label: %s" % self.labels[self.image_label_key])

        self.image_capture_count = {}

    def image_callback(self, image_msg):
        if not self.capture_images:
            return
        try:
            # Convert your ROS Image message to OpenCV2
            cv2_img = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
        except CvBridgeError, e:
            rospy.logerr(e)
            return

        timestamp = image_msg.header.stamp
        key_name = self.labels[self.image_label_key]
        if key_name not in self.image_capture_count:
            self.image_capture_count[key_name] = 0
        self.image_capture_count[key_name] += 1

        path = os.path.join(self.save_dir, key_name, str(timestamp)) + ".png"
        dir = os.path.dirname(path)
        if not os.path.isdir(dir):
            os.makedirs(dir)

        rospy.loginfo("Saving #%s to %s" % (self.image_capture_count[key_name], path))
        cv2.imwrite(path, cv2_img)

    def camera_callback(self, image_msg, depth_msg, info_msg):
        if not self.capture_images:
            return
        try:
            # Convert your ROS Image message to OpenCV2
            cv2_img = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
        except CvBridgeError, e:
            rospy.logerr(e)
            return

        try:
            cv_depth = self.bridge.imgmsg_to_cv2(depth_msg, "passthrough")
        except CvBridgeError as e:
            rospy.logerr(e)
            return

        timestamp = image_msg.header.stamp
        path = os.path.join(self.save_dir, key_name, str(timestamp)) + ".png"
        dir = os.path.dirname(path)
        if not os.path.isdir(dir):
            os.makedirs(dir)

        cv2.imwrite(path, cv2_img)

    def keyboard_callback(self, msg):
        keycode = msg.data
        if 0 <= keycode < 0x100:
            keyval = chr(keycode)

            if keyval == " ":
                self.capture_images = not self.capture_images
                if self.capture_images:
                    rospy.loginfo("Capturing images")
                else:
                    rospy.loginfo("Stopping capture")
            elif keyval in self.labels:
                self.image_label_key = keyval
                rospy.loginfo("Setting label to %s" % self.labels[self.image_label_key])

    def run(self):
        rospy.spin()


if __name__ == "__main__":
    node = ImageLogger()
    try:
        node.run()

    except rospy.ROSInterruptException:
        pass

    finally:
        rospy.loginfo("Exiting %s node" % node.node_name)
