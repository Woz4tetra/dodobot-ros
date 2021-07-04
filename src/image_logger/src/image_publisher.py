#!/usr/bin/python3

from __future__ import print_function

import os
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

# Instantiate CvBridge
bridge = CvBridge()
publish_rate = -1
image_path = ""
image_pub = None

def publish_image(image):
    try:
        image_msg = bridge.cv2_to_imgmsg(image, "bgr8")
    except CvBridgeError as e:
        print(e)
    else:
        image_pub.publish(image_msg)

def main():
    global image_pub, image_path, publish_rate

    rospy.init_node("image_logger")
    publish_rate = rospy.get_param("~publish_rate", -1)
    image_path = rospy.get_param("~image_path", "./image.jpg")
    image_topic = rospy.get_param("~image_topic", "/image")
    image_pub = rospy.Publisher(image_topic, Image, queue_size=1)

    image = cv2.imread(image_path)
    image = image[0:image.shape[0] - 20, 0:image.shape[1]]
    print(image.shape)

    if publish_rate > 0.0:
        clock_rate = rospy.Rate(publish_rate)
        while not rospy.is_shutdown():
            print("Publishing image")
            publish_image(image)
            clock_rate.sleep()
    else:
        publish_image(image)
        while not rospy.is_shutdown():
            rospy.sleep(1.0)


if __name__ == "__main__":
    main()
