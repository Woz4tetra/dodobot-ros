#!/usr/bin/env python3
import time
import rospy

import tf2_ros

import cv2
import numpy as np

import torch
from torch.backends import cudnn
from yolov5.models.common import DetectMultiBackend
from yolov5.utils.torch_utils import select_device
from yolov5.utils.general import non_max_suppression, scale_coords, xyxy2xywh
from yolov5.utils.plots import Annotator, colors
from yolov5.utils.augmentations import letterbox

from vision_msgs.msg import Detection2DArray
from vision_msgs.msg import Detection2D
from vision_msgs.msg import ObjectHypothesisWithPose
from vision_msgs.msg import Detection3DArray
from vision_msgs.msg import Detection3D

from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import CompressedImage

from image_geometry import PinholeCameraModel

from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped

import tf2_geometry_msgs

from visualization_msgs.msg import MarkerArray
from visualization_msgs.msg import Marker

from std_msgs.msg import ColorRGBA

from message_filters import ApproximateTimeSynchronizer
from message_filters import TimeSynchronizer
from message_filters import Subscriber

from cv_bridge import CvBridge, CvBridgeError

from db_tools.transforms import lookup_transform
from db_tools.yolo.detector import YoloDetector


class DodobotYolo:
    def __init__(self):
        self.name = "db_yolo"
        rospy.init_node(
            self.name
        )

        self.model_device = rospy.get_param("~model_device", "0")
        self.model_path = rospy.get_param("~model_path", "./yolov5s.pt")
        self.image_width_param = rospy.get_param("~image_width_param", "/camera/realsense2_camera/color_width")
        self.image_height_param = rospy.get_param("~image_height_param", "/camera/realsense2_camera/color_height")
        self.image_width = rospy.get_param(self.image_width_param, 960)
        self.image_height = rospy.get_param(self.image_height_param, 540)
        self.publish_overlay = rospy.get_param("~publish_overlay", True)
        self.confidence_threshold = rospy.get_param("~confidence_threshold", 0.25)
        self.nms_iou_threshold = rospy.get_param("~nms_iou_threshold", 0.45)
        self.max_detections = rospy.get_param("~max_detections", 100)  # maximum detections per image
        self.use_depth = rospy.get_param("~use_depth", True)
        self.z_depth_estimations = rospy.get_param("~z_depth_estimations", None)
        if self.z_depth_estimations is None:
            self.z_depth_estimations = {}
        self.base_frame = rospy.get_param("~base_frame", "base_link")
        marker_persistance_s = rospy.get_param("~marker_persistance", 0.5)
        self.marker_persistance = rospy.Duration(marker_persistance_s)
        self.bounding_box_border_px = rospy.get_param("~bounding_box_border_px", 10)
        self.report_loop_times = rospy.get_param("~report_loop_times", True)
        self.sync_method = rospy.get_param("~sync_method", "approx_sync")
        self.publish_delayed_image = rospy.get_param("~publish_delayed_image", True)

        self.yolo = YoloDetector(
            self.model_device, self.model_path, self.image_width, self.image_height,
            self.confidence_threshold, self.nms_iou_threshold, self.max_detections,
            self.report_loop_times, self.publish_overlay
        )

        self.label_colors = {}
        self.camera_model = None
        self.detection_result = Detection3DArray()
        self.marker_colors = {}
        self.timing_report = ""
        self.last_depth_msg = None
        self.time_sync = None
        self.color_image_sub = None
        self.depth_image_sub = None

        self.bridge = CvBridge()

        if self.use_depth:
            if self.sync_method == "approx_sync":
                rospy.loginfo("Synchronizing using approximate sync method")
                self.color_image_sub = self.make_color_sync_sub()
                self.depth_image_sub = self.make_depth_sync_sub()
                self.time_sync = ApproximateTimeSynchronizer([self.color_image_sub, self.depth_image_sub], queue_size=1, slop=0.075)
                self.time_sync.registerCallback(self.rgbd_callback)
            elif self.sync_method == "exact_sync":
                rospy.loginfo("Synchronizing using exact sync method")
                self.color_image_sub = self.make_color_sync_sub()
                self.depth_image_sub = self.make_depth_sync_sub()
                self.time_sync = TimeSynchronizer([self.color_image_sub, self.depth_image_sub], queue_size=1)
                self.time_sync.registerCallback(self.rgbd_callback)
            elif self.sync_method == "last_depth":
                rospy.loginfo("Synchronizing using last depth image method")
                self.color_image_sub = self.make_color_sub()
                self.depth_image_sub = self.make_depth_sub()
            else:
                rospy.loginfo("Not synchronizing depth (Unknown method: %s)" % self.sync_method)
                self.color_image_sub = self.make_color_sub()
        else:
            self.color_image_sub = self.make_color_sub()

        self.color_info_sub = rospy.Subscriber("color/camera_info", CameraInfo, self.info_callback, queue_size=5)
        
        self.overlay_pub = rospy.Publisher("overlay", Image, queue_size=1)
        # self.overlay_compressed_pub = rospy.Publisher("overlay/compressed", CompressedImage, queue_size=1)
        self.delayed_image_pub = rospy.Publisher("delayed_image", Image, queue_size=1)
        self.detections_pub = rospy.Publisher("detections", Detection3DArray, queue_size=25)
        self.markers_pub = rospy.Publisher("markers", MarkerArray, queue_size=25)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        rospy.loginfo("%s is ready" % self.name)
    
    def make_color_sub(self):
        return rospy.Subscriber("color/image_raw", Image, self.image_callback, queue_size=1, buff_size=2<<31)

    def make_depth_sub(self):
        return rospy.Subscriber("depth/image_raw", Image, self.depth_callback, queue_size=1, buff_size=2<<34)

    def make_color_sync_sub(self):
        # check out these forum posts about this solution:
        # https://answers.ros.org/question/50112/unexpected-delay-in-rospy-subscriber/
        # https://answers.ros.org/question/220502/image-subscriber-lag-despite-queue-1/?answer=220505?answer=220505#post-id-220505
        return Subscriber("color/image_raw", Image, buff_size=2<<31)

    def make_depth_sync_sub(self):
        return Subscriber("depth/image_raw", Image, buff_size=2<<31)

    def info_callback(self, msg):
        self.camera_model = PinholeCameraModel()
        self.camera_model.fromCameraInfo(msg)
        self.color_info_sub.unregister()  # only use the first message
        rospy.loginfo("Camera model loaded")

    def rgbd_callback(self, color_msg, depth_msg):
        self.compute_detections(color_msg, depth_msg)
    
    def depth_callback(self, depth_msg):
        self.last_depth_msg = depth_msg

    def image_callback(self, msg):
        self.compute_detections(msg, self.last_depth_msg)
    
    def compute_detections(self, color_msg, depth_msg):
        t_start = time.time()
        color_image = self.get_color_cv_image(color_msg)
        if color_image is None:
            return
        if depth_msg is not None:
            depth_image = self.get_depth_cv_image(depth_msg)
            if depth_image is None:
                return
        else:
            depth_image = None
        t_detect_start = time.time()
        if self.report_loop_times:
            self.timing_report = "------\n"
        detection_2d_arr_msg = self.get_detections_from_color(color_image)
        if self.report_loop_times:
            self.timing_report += self.yolo.timing_report
        t_detect = time.time()
        detection_3d_arr_msg = Detection3DArray()
        detection_3d_arr_msg.header = detection_2d_arr_msg.header

        for detection_2d_msg in detection_2d_arr_msg.detections:
            t0 = time.time()
            color = self.get_detection_color(color_image, detection_2d_msg)
            self.marker_colors[detection_2d_msg.results[0].id] = color
            t1 = time.time()
            if depth_image is not None:
                z_dist = self.get_depth_from_detection(depth_image, detection_2d_msg)
            else:
                z_dist = 1.0
            t2 = time.time()
            detection_3d_msg = self.detection_2d_to_3d(detection_2d_msg, z_dist)
            t3 = time.time()
            if detection_3d_msg is None:
                continue
            self.tf_detection_pose_to_robot(detection_3d_msg)
            t4 = time.time()
            if self.report_loop_times:
                label, count = self.get_detection_label(detection_2d_msg)
                self.timing_report += "\t%s-%s\n" % (label, count)
                self.timing_report += "\t\tget_detection_color: %0.4f\n" % (t1 - t0)
                self.timing_report += "\t\tget_depth_from_detection: %0.4f\n" % (t2 - t1)
                self.timing_report += "\t\tdetection_2d_to_3d: %0.4f\n" % (t3 - t2)
                self.timing_report += "\t\ttf_detection_pose_to_robot: %0.4f\n" % (t4 - t3)
            detection_3d_arr_msg.detections.append(detection_3d_msg)
        
        self.detection_result = detection_3d_arr_msg

        t_end = time.time()
        if self.report_loop_times:
            self.timing_report += "detect: %0.4f\n" % (t_detect - t_detect_start)
            self.timing_report += "total: %0.4f\n" % (t_end - t_start)
            self.timing_report += "since image start: %0.4f\n" % (t_start - color_msg.header.stamp.to_sec())
            self.timing_report += "since image end: %0.4f\n" % (t_end - color_msg.header.stamp.to_sec())
            rospy.loginfo_throttle(0.5, self.timing_report)
        self.detections_pub.publish(detection_3d_arr_msg)
        if self.publish_delayed_image:
            self.delayed_image_pub.publish(color_msg)
    
    def get_color_cv_image(self, msg):
        try:
            return self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
            return None

    def get_depth_cv_image(self, msg):
        try:
            return self.bridge.imgmsg_to_cv2(msg, "passthrough")
        except CvBridgeError as e:
            rospy.logerr(e)
            return None

    def get_detections_from_color(self, color_image):
        detection_arr_msg, overlay_image = self.detect(color_image)
        if self.publish_overlay and overlay_image is not None:
            try:
                # compressed_msg = CompressedImage()
                # compressed_msg.header.stamp = rospy.Time.now()
                # compressed_msg.format = "jpeg"
                # compressed_msg.data = np.array(cv2.imencode('.jpg', overlay_image)[1]).tobytes()
                # self.overlay_compressed_pub.publish(compressed_msg)
                
                overlay_msg = self.bridge.cv2_to_imgmsg(overlay_image, encoding="bgr8")
                self.overlay_pub.publish(overlay_msg)
            except TypeError as e:
                rospy.logerr("Exception occurred while converting frame: %s. %s" % (e, overlay_image.shape))
        return detection_arr_msg

    def get_depth_from_detection(self, depth_image, detection_2d_msg):
        depth = self.get_bbox_mean(depth_image, detection_2d_msg)[0]
        depth /= 1000.0  # depth image encoded in mm
        return depth
    
    def get_detection_color(self, color_image, detection_2d_msg):
        # return self.get_color_with_mask(color_image, detection_2d_msg)
        return self.get_color_with_center_px(color_image, detection_2d_msg)
        # return ColorRGBA(1.0, 0.0, 0.0, 1.0)
    
    def get_color_with_center_px(self, color_image, detection_2d_msg):
        center_x = int(detection_2d_msg.bbox.center.x)
        center_y = int(detection_2d_msg.bbox.center.y)
        color_raw = color_image[center_y, center_x]
        b = color_raw[0] / 255.0
        g = color_raw[1] / 255.0
        r = color_raw[2] / 255.0
        return ColorRGBA(r, g, b, 1.0)

    def get_color_with_mask(self, color_image, detection_2d_msg):
        label, count = self.get_detection_label(detection_2d_msg)
        if label not in self.label_colors:
            self.label_colors[label] = [None, rospy.Time.now()]
        color, timestamp = self.label_colors[label]
        if color is None or rospy.Time.now() - timestamp > rospy.Duration(1.0):
            color_raw = self.get_bbox_mean(color_image, detection_2d_msg)
            b = color_raw[0] / 255.0
            g = color_raw[1] / 255.0
            r = color_raw[2] / 255.0
            color = ColorRGBA(r, g, b, 1.0)
            self.label_colors[label][0] = color
            self.label_colors[label][1] = rospy.Time.now()
        return color

    def get_bbox_mean(self, image, detection_2d_msg):
        circle_mask = np.zeros(image.shape[0:2], np.uint8)
        radius = int(min(detection_2d_msg.bbox.size_x, detection_2d_msg.bbox.size_y) / 2.0)
        mask_radius = max(radius - self.bounding_box_border_px, 1)
        center = (int(detection_2d_msg.bbox.center.x), int(detection_2d_msg.bbox.center.y))
        circle_mask = cv2.circle(circle_mask, center, mask_radius, (255, 255, 255), cv2.FILLED)
        if len(image.shape) > 2 and image.shape[2] == 3:
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        else:
            gray = image
        _, nonzero_mask = cv2.threshold(gray, 0, np.max(image), cv2.THRESH_BINARY)
        nonzero_mask = nonzero_mask.astype(np.uint8)
        target_mask = cv2.bitwise_and(circle_mask, nonzero_mask)
        return cv2.mean(image, target_mask)

    def detection_2d_to_3d(self, detection_2d_msg, z_dist):
        center_point, box_size = self.get_detection_3d_box(detection_2d_msg, z_dist)
        if center_point is None or box_size is None:
            return None

        detection_3d_msg = Detection3D()
        detection_3d_msg.header.stamp = detection_2d_msg.header.stamp
        detection_3d_msg.results = detection_2d_msg.results

        pose = Pose()
        pose.position.x = center_point[0]
        pose.position.y = center_point[1]
        pose.position.z = center_point[2]
        pose.orientation.w = 1.0
        pose.orientation.x = 0.0
        pose.orientation.y = 0.0
        pose.orientation.z = 0.0

        detection_3d_msg.header.frame_id = self.camera_model.tfFrame()
        detection_3d_msg.results[0].pose.pose = pose
        detection_3d_msg.bbox.center = pose
        detection_3d_msg.bbox.size.x = box_size[0]
        detection_3d_msg.bbox.size.y = box_size[1]
        detection_3d_msg.bbox.size.z = box_size[2]

        return detection_3d_msg

    def tf_detection_pose_to_robot(self, detection_3d_msg):
        transform = lookup_transform(self.tf_buffer, self.base_frame, detection_3d_msg.header.frame_id)
        if transform is None:
            rospy.logwarn_throttle(1.0, "Can't transform detection to robot frame. Skipping")
            return
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = detection_3d_msg.header.frame_id
        pose_stamped.pose = detection_3d_msg.results[0].pose.pose
        robot_pose = tf2_geometry_msgs.do_transform_pose(pose_stamped, transform)
        detection_3d_msg.results[0].pose.pose = robot_pose.pose
        detection_3d_msg.bbox.center = robot_pose.pose
        detection_3d_msg.header = robot_pose.header

    def add_detection_to_marker_array(self, marker_array, detection_3d_msg, color):
        sphere_marker = self.make_marker(detection_3d_msg, color)
        text_marker = self.make_marker(detection_3d_msg, color)

        label, count = self.get_detection_label(detection_3d_msg)

        sphere_marker.type = Marker.SPHERE
        sphere_marker.ns = "sphere_" + sphere_marker.ns

        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.ns = "text_" + sphere_marker.ns
        text_marker.text = "%s_%s|%0.1f" % (label, count, detection_3d_msg.results[0].score * 100)
        text_marker.scale.z = min(text_marker.scale.x, text_marker.scale.y)
        text_marker.scale.x = 0.0
        text_marker.scale.y = 0.0

        marker_array.markers.append(sphere_marker)
        marker_array.markers.append(text_marker)

    
    def make_marker(self, detection_3d_msg, color):
        marker = Marker()
        marker.action = Marker.ADD
        marker.pose = detection_3d_msg.results[0].pose.pose
        marker.header = detection_3d_msg.header
        marker.lifetime = self.marker_persistance
        label, count = self.get_detection_label(detection_3d_msg)
        marker.ns = label
        marker.id = count

        marker.scale.x = detection_3d_msg.bbox.size.x
        marker.scale.y = detection_3d_msg.bbox.size.y
        marker.scale.z = detection_3d_msg.bbox.size.z
        marker.color = color

        return marker
    
    def get_detection_label(self, detection_msg):
        return self.yolo.get_label(detection_msg.results[0].id)

    def get_detection_3d_box(self, detection_msg, z_dist):
        if self.camera_model is None:
            rospy.logerr_throttle(0.5, "No camera model has been loaded! Is the info topic publish messages?")
            return None, None
        center_x = int(detection_msg.bbox.center.x)
        center_y = int(detection_msg.bbox.center.y)
        ray = self.camera_model.projectPixelTo3dRay((center_x, center_y))
        x_dist = ray[0] * z_dist
        y_dist = ray[1] * z_dist

        label, count = self.get_detection_label(detection_msg)
        edge_point_x = center_x - int(detection_msg.bbox.size_x / 2.0)
        edge_point_y = center_y - int(detection_msg.bbox.size_y / 2.0)
        ray = self.camera_model.projectPixelTo3dRay((edge_point_x, edge_point_y))
        x_size = abs(ray[0] * z_dist - x_dist) * 2.0
        y_size = abs(ray[1] * z_dist - y_dist) * 2.0
        z_size = self.z_depth_estimations.get(label, 0.01)

        return (x_dist, y_dist, z_dist), (x_size, y_size, z_size)

    def detect(self, image):
        detections, overlay_image = self.yolo.detect(image)
        
        detection_arr_msg = Detection2DArray()
        detection_arr_msg.header.stamp = rospy.Time.now()
        for obj_id, (bndbox, confidence) in detections.items():
            detection_msg = Detection2D()
            detection_msg.header.stamp = detection_arr_msg.header.stamp
            box_width = bndbox[2] - bndbox[0]
            box_height = bndbox[3] - bndbox[1]
            detection_msg.bbox.center.x = bndbox[0] + box_width / 2.0
            detection_msg.bbox.center.y = bndbox[1] + box_height / 2.0
            detection_msg.bbox.size_x = box_width
            detection_msg.bbox.size_y = box_height
            obj_with_pose = ObjectHypothesisWithPose()
            obj_with_pose.id = obj_id
            obj_with_pose.score = confidence
            detection_msg.results.append(obj_with_pose)

            detection_arr_msg.detections.append(detection_msg)

        return detection_arr_msg, overlay_image


    def run(self):
        clock_rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            markers = MarkerArray()
            for detection_3d_msg in self.detection_result.detections:
                if rospy.Time.now() - detection_3d_msg.header.stamp > self.marker_persistance:
                    continue
                color = self.marker_colors[detection_3d_msg.results[0].id]
                self.add_detection_to_marker_array(markers, detection_3d_msg, color)
            self.markers_pub.publish(markers)
            clock_rate.sleep()


def main():
    node = DodobotYolo()
    try:
        node.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        rospy.loginfo("Exiting %s node" % node.name)


if __name__ == "__main__":
    main()
