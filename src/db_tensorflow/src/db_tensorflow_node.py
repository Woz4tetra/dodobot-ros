#!/usr/bin/env python3
import rospy

import time
import cv2
import numpy as np

import tensorflow
from object_detection.utils import label_map_util
from object_detection.utils import visualization_utils as viz_utils

import message_filters
from sensor_msgs.msg import Image, CameraInfo

from cv_bridge import CvBridge, CvBridgeError


class DodobotTensorflow:
    def __init__(self):
        self.node_name = "db_tensorflow"
        rospy.init_node(
            self.node_name
            # disable_signals=True
            # log_level=rospy.DEBUG
        )
        # rospy.on_shutdown(self.shutdown)

        self.image_topic_name = rospy.get_param("~image_topic", "color/image_raw")
        self.depth_topic_name = rospy.get_param("~depth_topic", "depth/image_raw")
        self.info_topic_name = rospy.get_param("~camera_info_topic", "color/camera_info")

        self.visualization_topic_name = rospy.get_param("~visualization_topic", "labeled_image")

        self.labels_path = rospy.get_param("~labels_path", "annotations/label_map.pbtxt")
        self.model_path = rospy.get_param("~model_path", "models/dodobot_objects_ssd_resnet50_v1_fpn")

        self.min_score_threshold = rospy.get_param("~min_score_threshold", 0.3)
        self.max_boxes_to_draw = rospy.get_param("~max_boxes_to_draw", 20)

        self.image_sub = message_filters.Subscriber(self.image_topic_name, Image)
        self.depth_sub = message_filters.Subscriber(self.depth_topic_name, Image)
        self.info_sub = message_filters.Subscriber(self.info_topic_name, CameraInfo)

        self.time_sync_sub = message_filters.TimeSynchronizer([self.image_sub, self.depth_sub, self.info_sub], 10)
        self.time_sync_sub.registerCallback(self.rgbd_callback)

        self.visualization_image_pub = rospy.Publisher(self.visualization_topic_name, Image, queue_size=1)

        self.bridge = CvBridge()

        self.detect_fn, self.category_index = self.generate_detect_fn()
    
    def generate_detect_fn(self):
        category_index = label_map_util.create_category_index_from_labelmap(self.labels_path, use_display_name=True)

        rospy.loginfo("Loading model...")
        start_time = time.time()

        # Load saved model and build the detection function
        detect_fn = tensorflow.saved_model.load(self.model_path)

        end_time = time.time()
        elapsed_time = end_time - start_time
        rospy.loginfo("Done! Took %0.2f seconds" % (elapsed_time))

        return detect_fn, category_index

    def rgbd_callback(self, color_image, depth_image, camera_info):
        try:
            # Convert ROS Image message to numpy array
            color_image_np = self.bridge.imgmsg_to_cv2(color_image, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
            return
        input_tensor = tensorflow.convert_to_tensor(color_image_np)
        input_tensor = input_tensor[tensorflow.newaxis, ...]

        detections = self.detect_fn(input_tensor)

        num_detections = int(detections.pop("num_detections"))
        detections = {key: value[0, :num_detections].numpy() for key, value in detections.items()}
        detections["num_detections"] = num_detections

        # detection_classes should be ints.
        detections["detection_classes"] = detections["detection_classes"].astype(np.int64)

        image_with_detections = color_image_np.copy()

        self.publish_visualization(detections, image_with_detections)

    def publish_visualization(self, detections, image_with_detections):
        if self.visualization_image_pub.get_num_connections() == 0:
            return
        viz_utils.visualize_boxes_and_labels_on_image_array(
            image_with_detections,
            detections["detection_boxes"],
            detections["detection_classes"],
            detections["detection_scores"],
            self.category_index,
            use_normalized_coordinates=True,
            max_boxes_to_draw=self.max_boxes_to_draw,
            min_score_thresh=self.min_score_threshold,
            agnostic_mode=False
        )

        try:
            visualize_image_msg = bridge.cv2_to_imgmsg(image_with_detections, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
            return
        self.visualization_image_pub.publish(visualize_image_msg)


    def run(self):
        pass

if __name__ == "__main__":
    try:
        node = DodobotTensorflow()
        node.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        rospy.loginfo("Exiting central_planning node")
