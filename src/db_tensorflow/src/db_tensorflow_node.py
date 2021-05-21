#!/usr/bin/env python3
import os
import rospy

import cv2
import time
import copy
import pickle
import traceback
import numpy as np

import tensorflow
from object_detection.utils import label_map_util
from object_detection.utils import visualization_utils as viz_utils

from dynamic_reconfigure.server import Server
from db_tensorflow.cfg import DetectConfig

import message_filters
from sensor_msgs.msg import Image, CameraInfo

from image_geometry import PinholeCameraModel

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovariance
from geometry_msgs.msg import Vector3 

from vision_msgs.msg import Detection2D
from vision_msgs.msg import Detection2DArray
from vision_msgs.msg import ObjectHypothesisWithPose

from visualization_msgs.msg import MarkerArray
from visualization_msgs.msg import Marker

from std_msgs.msg import ColorRGBA

import tf2_ros
import tf2_geometry_msgs

import ctypes
# a thread gets killed improperly within CvBridge without this causing segfaults
libgcc_s = ctypes.CDLL('libgcc_s.so.1')

from cv_bridge import CvBridge, CvBridgeError


class RollingAvgRateLogger:
    def __init__(self, rolling_avg_window):
        self.rolling_avg_window = rolling_avg_window
        self.data = np.zeros(self.rolling_avg_window)
        self.index = 0
    
    def append(self, value):
        self.data[self.index] = value
        self.index += 1
        if self.index >= len(self.data):
            self.index = 0

    def rate(self):
        rate = 1.0 / np.mean(self.data)
        return rate
    
    def avg(self):
        return np.mean(self.data)


class BoxDescription:
    def __init__(self):
        self.class_index = 0
        self.class_name = ""
        self.score = 0.0
        self.box = (0.0, 0.0, 0.0, 0.0)
        self.xcenter = 0.0
        self.ycenter = 0.0
        self.xsize = 0.0
        self.ysize = 0.0
        self.obj_radius = 0.0
        self.detect_radius = 0.0

        self.pose_stamped = PoseStamped()
        self.xedge = 0.0
        self.yedge = 0.0
        self.zedge = 0.0


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

        self.labels_path = rospy.get_param("~labels_path", "annotations/label_map.pbtxt")
        self.model_path = rospy.get_param("~model_path", "models/dodobot_objects_ssd_resnet50_v1_fpn")

        self.min_score_threshold = rospy.get_param("~min_score_threshold", 0.3)
        self.max_boxes_to_draw = rospy.get_param("~max_boxes_to_draw", 20)
        self.bounding_box_border_px = rospy.get_param("~bounding_box_border_px", 30)
        self.z_size_estimations = rospy.get_param("~z_size_estimations", None)
        marker_colors = rospy.get_param("~marker_colors", None)
        self.publish_in_robot_frame = rospy.get_param("~publish_in_robot_frame", True)
        self.robot_frame = rospy.get_param("~robot_frame", "base_link")
        self.marker_persistance = rospy.get_param("~marker_persistance_s", 0.5)
        self.default_z_size = rospy.get_param("~default_z_size", 0.05)
        self.min_valid_z = rospy.get_param("~min_valid_z", 0.03)
        self.max_valid_z = rospy.get_param("~max_valid_z", 3.0)

        self.marker_persistance = rospy.Duration(self.marker_persistance)
        self.marker_colors = {name: ColorRGBA(*args) for name, args in marker_colors.items()}

        self.image_sub = message_filters.Subscriber(self.image_topic_name, Image)
        self.depth_sub = message_filters.Subscriber(self.depth_topic_name, Image)

        self.detect_fn = None
        self.category_index = None
        
        self.dyn_server = Server(DetectConfig, self.dyn_callback)


        self.time_sync_sub = message_filters.TimeSynchronizer([self.image_sub, self.depth_sub], 1)
        # self.time_sync_sub = message_filters.ApproximateTimeSynchronizer([self.image_sub, self.depth_sub], 10, 0.005)

        self.visualization_image_pub = rospy.Publisher("detect_overlay", Image, queue_size=1)

        self.detect_pub = rospy.Publisher("detections", Detection2DArray, queue_size=10)
        self.marker_pub = rospy.Publisher("obj_markers", MarkerArray, queue_size=25)

        self.bridge = CvBridge()
        self.camera_model = PinholeCameraModel()

        self.detect_rate_logger = RollingAvgRateLogger(10)
        self.update_rate_logger = RollingAvgRateLogger(10)
        self.camera_rate_logger = RollingAvgRateLogger(10)
        self.depth_rate_logger = RollingAvgRateLogger(10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # with open("/home/ben/detection.pkl", 'rb') as file:
        #     self.dummy_detections = pickle.load(file)

        self.generate_detect_fn()
        
        self.time_sync_sub.registerCallback(self.rgbd_callback)
        self.info_sub = rospy.Subscriber(self.info_topic_name, CameraInfo, self.camera_info_callback, queue_size=5)

        rospy.loginfo("%s init done" % self.node_name)
    
    def generate_detect_fn(self):
        if not os.path.isfile(self.labels_path):
            raise FileNotFoundError("Labels path doesn't exist: %s" % self.labels_path)

        if not os.path.isdir(self.model_path):
            raise FileNotFoundError("Model path doesn't exist: %s" % self.model_path)

        category_index = label_map_util.create_category_index_from_labelmap(self.labels_path, use_display_name=True)

        rospy.loginfo("Loading model...")
        start_time = time.time()
        # Load saved model and build the detection function
        detect_fn = tensorflow.saved_model.load(self.model_path)
        # detect_fn = None

        end_time = time.time()
        elapsed_time = end_time - start_time
        rospy.loginfo("Model took %0.2f seconds to load" % (elapsed_time))

        self.detect_fn = detect_fn
        self.category_index = category_index

        rospy.loginfo("Priming the pump...")
        color_image_np = np.zeros((540, 960, 3), dtype=np.uint8)
        start_time = time.time()
        self.run_detection(color_image_np)
        end_time = time.time()
        elapsed_time = end_time - start_time

        rospy.loginfo("Priming took %0.2f seconds" % (elapsed_time))

    def rgbd_callback(self, color_image, depth_image):
        t0 = rospy.Time.now()
        detections = self.detection_pipeline(color_image)
        
        detect_array_msg = Detection2DArray()
        markers_msg = MarkerArray()

        for index in range(detections["num_detections"]):
            score = detections["detection_scores"][index]
            if score < self.min_score_threshold:
                continue
            box = detections["detection_boxes"][index]
            class_index = detections["detection_classes"][index]
            desc = self.depth_pipeline(box, score, class_index, depth_image)

            if desc.pose_stamped is None or not (self.min_valid_z <= desc.pose_stamped.pose.position.z <= self.max_valid_z):
                continue
            
            detect_msg = self.get_detect_msg(desc)
            
            if self.publish_in_robot_frame:
                new_pose = self.object_to_robot_frame(desc.pose_stamped, color_image.header)
                if new_pose is None:
                    continue
                else:
                    desc.pose_stamped = new_pose
            
            detection_pose = PoseWithCovariance()
            detection_pose.pose = desc.pose_stamped.pose
            detect_msg.results[0].pose = detection_pose
            detect_msg.header = desc.pose_stamped.header

            markers = self.make_markers(desc)

            detect_array_msg.detections.append(detect_msg)
            markers_msg.markers.extend(markers)

        detect_array_msg.header.stamp = color_image.header.stamp

        self.detect_pub.publish(detect_array_msg)
        self.marker_pub.publish(markers_msg)

        t1 = rospy.Time.now()
        self.update_rate_logger.append((t1 - t0).to_sec())
        self.camera_rate_logger.append((t0 - color_image.header.stamp).to_sec())
        rospy.loginfo_throttle(10, "Detection rate avg: %0.3f Hz" % self.update_rate_logger.rate())
        rospy.loginfo_throttle(10, "Camera delay avg: %0.3fs" % self.camera_rate_logger.avg())
    
    def camera_info_callback(self, camera_info):
        self.camera_model.fromCameraInfo(camera_info)

    def run_detection(self, color_image_np):
        input_tensor = tensorflow.convert_to_tensor(color_image_np)
        input_tensor = input_tensor[tensorflow.newaxis, ...]

        t0 = rospy.Time.now()
        # if self.detect_fn is None:
        #     detections = copy.deepcopy(self.dummy_detections)
        # else:
        detections = self.detect_fn(input_tensor)
            # with open("/home/ben/detection.pkl", 'wb') as file:
            #     pickle.dump(detections, file)
        t1 = rospy.Time.now()
        self.detect_rate_logger.append((t1 - t0).to_sec())

        rospy.loginfo_throttle(10, "Detection rate avg: %0.3f Hz" % self.detect_rate_logger.rate())

        return detections

    def detection_pipeline(self, color_image):
        try:
            # Convert ROS Image message to numpy array
            color_image_np = self.bridge.imgmsg_to_cv2(color_image, "rgb8")
        except CvBridgeError as e:
            rospy.logerr(e)
            return
        
        detections = self.run_detection(color_image_np)

        num_detections = int(detections.pop("num_detections"))
        detections = {key: value[0, :num_detections].numpy() for key, value in detections.items()}
        detections["num_detections"] = num_detections

        # detection_classes should be ints.
        detections["detection_classes"] = detections["detection_classes"].astype(np.int64)

        image_with_detections = color_image_np.copy()

        self.publish_visualization(detections, image_with_detections)

        return detections
    
    def depth_pipeline(self, box, score, class_index, depth_image):
        t0 = time.time()
        class_name = self.category_index[class_index]["name"]
        ymin, xmin, ymax, xmax = box

        # depth image dimensions should be the same as the color image because
        # the depth image has been registered/shifted to be overlaid on top of the color image
        ymin *= depth_image.height
        ymax *= depth_image.height
        xmin *= depth_image.width
        xmax *= depth_image.width

        xcenter = (xmax + xmin) / 2.0
        ycenter = (ymax + ymin) / 2.0
        xsize = xmax - xmin
        ysize = ymax - ymin
        obj_radius = min(xsize, ysize) / 2.0
        detect_radius = max(obj_radius - self.bounding_box_border_px, 1)

        desc = BoxDescription()
        desc.class_index = class_index
        desc.class_name = class_name
        desc.score = score
        desc.box = (ymin, xmin, ymax, xmax)
        desc.xcenter = xcenter
        desc.ycenter = ycenter
        desc.xsize = xsize
        desc.ysize = ysize
        desc.obj_radius = obj_radius
        desc.detect_radius = detect_radius

        if self.z_size_estimations is not None:
            zsize = self.z_size_estimations.get(class_name, self.default_z_size)
        else:
            zsize = self.default_z_size
        z_depth = self.z_depth_from_image(depth_image, desc)
        if z_depth is None:
            return None
        z_depth += zsize / 2.0  # move to the rough center of the object
        ray = self.camera_model.projectPixelTo3dRay((desc.xcenter, desc.ycenter))

        x_pos = ray[0] * z_depth
        y_pos = ray[1] * z_depth

        pose = PoseStamped()
        pose.header.frame_id = self.camera_model.tfFrame()
        pose.header.stamp = depth_image.header.stamp
        pose.pose.position.x = x_pos
        pose.pose.position.y = y_pos
        pose.pose.position.z = z_depth
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = 0.0
        pose.pose.orientation.w = 1.0

        edge_point_x = desc.xcenter - xsize / 2.0
        edge_point_y = desc.ycenter - ysize / 2.0
        ray = self.camera_model.projectPixelTo3dRay((edge_point_x, edge_point_y))

        desc.pose_stamped = pose
        desc.xedge = abs(ray[0] * z_depth - x_pos) * 2.0
        desc.yedge = abs(ray[1] * z_depth - y_pos) * 2.0
        desc.zedge = zsize

        t1 = time.time()
        self.depth_rate_logger.append(t1 - t0)
        rospy.loginfo_throttle(10, "Depth pipeline rate avg: %0.3f Hz" % self.depth_rate_logger.rate())

        return desc
    
    def z_depth_from_image(self, depth_image, desc: BoxDescription):
        try:
            # Convert ROS Image message to numpy array
            depth_image_np = self.bridge.imgmsg_to_cv2(depth_image, desired_encoding="passthrough")
        except CvBridgeError as e:
            rospy.logerr(e)
            return None
        depth_image_np = depth_image_np.astype(np.float32)
        circle_mask = np.zeros(depth_image_np.shape, np.uint8)
        cv2.circle(circle_mask, (int(desc.xcenter), int(desc.ycenter)), int(desc.detect_radius), (255, 255, 255), cv2.FILLED)
        nonzero_mask = (depth_image_np > 0.0).astype(np.uint8)
        target_mask = cv2.bitwise_and(circle_mask, nonzero_mask)
        z_depth = cv2.mean(depth_image_np, target_mask)[0]
        z_depth /= 1000.0
        return z_depth

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
            visualize_image_msg = self.bridge.cv2_to_imgmsg(image_with_detections, "rgb8")
        except CvBridgeError as e:
            rospy.logerr(e)
            return
        self.visualization_image_pub.publish(visualize_image_msg)

    def get_detect_msg(self, desc: BoxDescription):
        detect_msg = Detection2D()
        detect_msg.bbox.size_x = desc.xsize
        detect_msg.bbox.size_y = desc.ysize
        detect_msg.bbox.center.x = desc.xcenter
        detect_msg.bbox.center.y = desc.ycenter
        hyp = ObjectHypothesisWithPose()

        hyp.id = desc.class_index
        hyp.score = desc.score
        detect_msg.results.append(hyp)
        return detect_msg

    def object_to_robot_frame(self, pose_stamped, image_header):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.robot_frame,
                image_header.frame_id,
                image_header.stamp,
                rospy.Duration(1.0)
            )
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn("Failed to look up %s to %s. %s" % (self.robot_frame, image_header.frame_id, e))
            return None
        pose_robot_frame = tf2_geometry_msgs.do_transform_pose(pose_stamped, transform)
        return pose_robot_frame

    def dyn_callback(self, config, level):
        rospy.loginfo("Reconfigure Request: %s. Level: %s" % (str(config), level))
        if level >= 0:
            rospy.loginfo("Reconfiguring")
            self.min_score_threshold = config["min_score_threshold"]
            self.max_boxes_to_draw = config["max_boxes_to_draw"]
            self.bounding_box_border_px = config["bounding_box_border_px"]
            self.min_valid_z = config["min_valid_z"]
            self.max_valid_z = config["max_valid_z"]
        return config

    def make_markers(self, desc: BoxDescription):
        sphere_marker = self.make_marker_base(desc)
        text_marker = self.make_marker_base(desc)

        sphere_marker.type = Marker.SPHERE
        sphere_marker.ns = "sphere_" + sphere_marker.ns

        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.ns = "text_" + text_marker.ns
        text_marker.text = desc.class_name + "_" + str(desc.class_index)
        text_marker.scale.x = 0.0
        text_marker.scale.y = 0.0

        return [sphere_marker, text_marker]

    def make_marker_base(self, desc: BoxDescription):
        marker = Marker()
        marker.action = Marker.ADD
        marker.pose = desc.pose_stamped.pose
        marker.header = desc.pose_stamped.header
        marker.lifetime = self.marker_persistance
        marker.ns = desc.class_name
        marker.id = desc.class_index

        scale_vector = Vector3()
        scale_vector.x = desc.xedge
        scale_vector.y = desc.yedge
        scale_vector.z = desc.zedge
        marker.scale = scale_vector
        if self.marker_colors is not None and desc.class_name in self.marker_colors:
            marker.color = self.marker_colors[desc.class_name]
        else:
            marker.color = ColorRGBA(1.0, 1.0, 1.0, 1.0)

        return marker

    def run(self):
        rospy.spin()

if __name__ == "__main__":
    try:
        node = DodobotTensorflow()
        node.run()
    except rospy.ROSInterruptException:
        pass
    except BaseException as e:
        rospy.logerr("%s: %s\n%s" % (e.__class__.__name__, str(e), traceback.format_exc()))
    finally:
        rospy.loginfo("Exiting db_tensorflow node")
