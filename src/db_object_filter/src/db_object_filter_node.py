#!/usr/bin/env python3

import math
import rospy

import numpy as np

from vision_msgs.msg import Detection2DArray

from nav_msgs.msg import Odometry

from geometry_msgs.msg import PoseStamped, Quaternion

from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose
from geometry_msgs.msg import TransformStamped

import tf2_ros
import tf_conversions
import tf2_geometry_msgs

from db_filter_factory import FilterFactory


class ObjectFilterNode:
    def __init__(self):
        self.node_name = "db_object_filter"
        rospy.init_node(
            self.node_name,
            # disable_signals=True
            # log_level=rospy.DEBUG
        )
        # rospy.on_shutdown(self.shutdown_hook)

        self.class_labels = rospy.get_param("~class_labels", None)
        self.filter_frame = rospy.get_param("~filter_frame", "base_link")
        self.odom_frame = rospy.get_param("~odom_frame", "odom")
        self.global_frame = rospy.get_param("~global_frame", "map")

        self.show_particles = rospy.get_param("~publish_particles", True)

        twist_lookup_avg_interval = rospy.get_param("~twist_lookup_avg_interval", 1.0 / 30.0)
        self.twist_lookup_avg_interval = rospy.Duration(twist_lookup_avg_interval)
        self.initial_range = rospy.get_param("~initial_range", None)
        self.input_std = rospy.get_param("~input_std", None)
        self.meas_std_val = rospy.get_param("~meas_std_val", 0.007)
        self.stale_meas_std_val = rospy.get_param("~stale_meas_std_val", 0.07)
        self.num_particles = rospy.get_param("~num_particles", 100)

        self.input_std = rospy.get_param("~input_std", None)
        self.match_cov = rospy.get_param("~match_cov", 0.05)
        self.match_threshold = rospy.get_param("~match_threshold", 0.7)
        self.new_filter_threshold = rospy.get_param("~new_filter_threshold", 0.7)
        self.confident_filter_threshold = rospy.get_param("~confident_filter_threshold", 0.005)
        self.max_item_count = rospy.get_param("~max_item_count", None)
        self.stale_filter_time = rospy.get_param("~stale_filter_time", 3.0)

        assert self.class_labels is not None
        assert len(self.class_labels) > 0

        if self.max_item_count is None:
            self.max_item_count = {}

        if self.initial_range is None:
            self.initial_range = [1.0, 1.0, 1.0]
        if self.input_std is None:
            self.input_std = [0.007, 0.007, 0.007, 0.007]
        self.input_vector = np.zeros(len(self.input_std))

        self.factory = FilterFactory(
            self.class_labels,
            self.num_particles, self.meas_std_val, self.stale_meas_std_val, self.input_std, self.initial_range,
            self.match_cov, self.match_threshold, self.new_filter_threshold, self.max_item_count,
            self.confident_filter_threshold, self.stale_filter_time
        )
        self.prev_pf_time = rospy.Time.now().to_sec()

        self.detections_sub = rospy.Subscriber("detections", Detection2DArray, self.detections_callback, queue_size=25)
        self.odom_sub = rospy.Subscriber("odom", Odometry, self.odom_callback, queue_size=25)

        self.particles_pub = rospy.Publisher("pf_particles", PoseArray, queue_size=5)

        # self.tf_listener = tf.TransformListener()
        self.broadcaster = tf2_ros.TransformBroadcaster()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.prev_pose = None
        self.prev_transform = None
        self.zero_pose = PoseStamped()
        self.zero_pose.header.frame_id = self.filter_frame

        rospy.loginfo("%s init done" % self.node_name)

    def to_label(self, obj_id):
        return self.class_labels[obj_id]
        
    def detections_callback(self, msg):
        measurements = {}
        for detection in msg.detections:
            obj = detection.results[0]
            label = self.to_label(obj.id)
            obj_pose = obj.pose.pose
            measurement = np.array([
                obj_pose.position.x,
                obj_pose.position.y,
                obj_pose.position.z,
            ])
            if label not in measurements:
                measurements[label] = []
            measurements[label].append(measurement)
        self.factory.update(measurements)

    def odom_callback(self, msg):
        current_time = msg.header.stamp.to_sec()
        dt = current_time - self.prev_pf_time
        self.prev_pf_time = current_time
    
        self.input_vector[0] = -msg.twist.twist.linear.x * dt
        self.input_vector[3] = -msg.twist.twist.angular.z * dt
        self.factory.predict(self.input_vector)

    def update_stale_filter(self, obj_filter):
        obj_frame = obj_filter.get_name()
        if obj_filter.stale_measurement is None:
            transform = self.lookup_transform(self.global_frame, obj_frame)
            if transform is None:
                return
            obj_pose = PoseStamped()
            obj_pose.header.frame_id = self.global_frame
            obj_pose.pose.position.x = transform.transform.translation.x
            obj_pose.pose.position.y = transform.transform.translation.y
            obj_pose.pose.position.z = transform.transform.translation.z
            # objects only have meaningful XYZ. May add orientation in the future

            obj_filter.stale_measurement = obj_pose
            return 
        else:
            transform = self.lookup_transform(self.filter_frame, self.global_frame)
            stale_meas_pose = tf2_geometry_msgs.do_transform_pose(obj_filter.stale_measurement, transform)

            stale_measurement = np.array([
                stale_meas_pose.pose.position.x,
                stale_meas_pose.pose.position.y,
                stale_meas_pose.pose.position.z,
            ])

        # rospy.loginfo("Applying stale measurement to %s. [%0.4f, %0.4f, %0.4f]" % (
        #     obj_frame, stale_measurement[0], stale_measurement[1], stale_measurement[2])
        # )
        obj_filter.update(stale_measurement, is_stale=True)

    def lookup_transform(self, parent_link, child_link, time_window=None, timeout=None):
        """
        Call tf_buffer.lookup_transform. Return None if the look up fails
        """
        if time_window is None:
            time_window = rospy.Time(0)
        else:
            time_window = rospy.Time.now() - time_window

        if timeout is None:
            timeout = rospy.Duration(1.0)

        try:
            return self.tf_buffer.lookup_transform(parent_link, child_link, time_window, timeout)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn("Failed to look up %s to %s. %s" % (parent_link, child_link, e))
            return None
    
    def quat_to_yaw(self, quaternion):
        return tf_conversions.transformations.euler_from_quaternion((
            quaternion.x,
            quaternion.y,
            quaternion.z,
            quaternion.w
        ))[2]

    def publish_all_poses(self):
        for obj_filter in self.factory.iter_filters():
            mean = obj_filter.mean()
            name = obj_filter.get_name()

            msg = TransformStamped()
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = self.filter_frame
            msg.child_frame_id = name
            msg.transform.translation.x = mean[0]
            msg.transform.translation.y = mean[1]
            msg.transform.translation.z = mean[2]
            msg.transform.rotation.w = 1.0
            self.broadcaster.sendTransform(msg)
    
    def publish_particles(self):
        if self.particles_pub.get_num_connections() == 0:
            return
        particles_msg = PoseArray()
        particles_msg.header.frame_id = self.filter_frame
        particles_msg.header.stamp = rospy.Time.now()

        for obj_filter in self.factory.iter_filters():
            for particle in obj_filter.particles:
                pose_msg = Pose()
                pose_msg.position.x = particle[0]
                pose_msg.position.y = particle[1]
                pose_msg.position.z = particle[2]
                particles_msg.poses.append(pose_msg)

        self.particles_pub.publish(particles_msg)

    def run(self):
        rate = rospy.Rate(30.0)

        while True:
            rate.sleep()
            if rospy.is_shutdown():
                break
            
            self.factory.check_resample()
            for obj_filter in self.factory.iter_stale_filters():
                self.update_stale_filter(obj_filter)
            self.publish_all_poses()
            
            if self.show_particles:
                self.publish_particles()
        # rospy.spin()


if __name__ == "__main__":
    node = ObjectFilterNode()
    try:
        node.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        rospy.loginfo("Exiting %s node" % node.node_name)
