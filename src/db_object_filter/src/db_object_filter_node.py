#!/usr/bin/python

import rospy
import threading
from vision_msgs.msg import Detection2DArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose

from db_particle_filter import ParticleFilter


class ObjectFilterNode:
    def __init__(self):
        self.node_name = "db_object_filter"
        rospy.init_node(
            self.node_name
            # disable_signals=True
            # log_level=rospy.DEBUG
        )
        # rospy.on_shutdown(self.shutdown_hook)

        self.class_labels = rospy.get_param("~class_labels", None)
        self.filter_frame = rospy.get_param("~filter_frame", "base_link")

        self.initial_range = rospy.get_param("~initial_range", None)
        self.input_std = rospy.get_param("~input_std", None)
        self.meas_std_val = rospy.get_param("~meas_std_val", 0.007)
        self.num_particles = rospy.get_param("~num_particles", 250)

        assert self.class_labels is not None
        assert len(self.class_labels) > 0

        if self.initial_range is None:
            self.initial_range = [1.0, 1.0, 1.0]
        if self.input_std is None:
            self.input_std = [0.007, 0.007, 0.007, 0.007]
        self.initial_state = None

        self.pf = ParticleFilter(self.num_particles, self.meas_std_val)
        self.pf_lock = threading.Lock()

        self.detections_sub = rospy.Subscriber("detections", Detection2DArray, self.detections_callback, queue_size=25)
        self.odom_sub = rospy.Subscriber("odom", Odometry, self.odom_callback, queue_size=25)

        self.particles_pub = rospy.Publisher("pf_particles", PoseArray)
        self.pose_pub = rospy.Publisher("pf_pose", PoseStamped)
    
    def to_label(self, obj_id):
        return self.class_labels[obj_id]
        
    def detections_callback(self, msg):
        for detection in msg.detections:
            obj = detection.results[0]
            label_name = self.to_label(obj.id)
            if label_name != "blue_cut_sphere":
                continue
            obj_pose = obj.pose.pose
            measurement = [
                obj_pose.position.x,
                obj_pose.position.y,
                obj_pose.position.z,
            ]
            with self.pf_lock:
                if self.initial_state is None:
                    self.initial_state = measurement
                    self.pf.create_uniform_particles(self.initial_state, self.initial_range)
                    self.prev_pf_time = rospy.Time.now().to_sec()
                self.pf.update(measurement)

    def odom_callback(self, msg):
        if self.initial_state is None:
            return
        dt = msg.header.stamp.to_sec() - self.prev_pf_time
        self.prev_pf_time = msg.header.stamp.to_sec()

        input_vector = [
            -msg.twist.twist.linear.x,
            0.0,
            0.0,
            -msg.twist.twist.angular.z,
        ]
        with self.pf_lock:
            self.pf.predict(input_vector, self.input_std, dt)

    def publish_pose(self, pose_mean):
        pose_msg = PoseStamped()
        pose_msg.header.frame_id = self.filter_frame
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.pose.position.x = pose_mean[0]
        pose_msg.pose.position.y = pose_mean[1]
        pose_msg.pose.position.z = pose_mean[2]

        self.pose_pub.publish(pose_msg)
    
    def publish_particles(self, particles):
        particles_msg = PoseArray()
        particles_msg.header.frame_id = self.filter_frame
        particles_msg.header.stamp = rospy.Time.now()

        for particle in particles:
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
            with self.pf_lock:
                self.pf.check_resample()

                self.publish_pose(self.pf.mean())
                self.publish_particles(self.pf.particles)
        # rospy.spin()


if __name__ == "__main__":
    node = ObjectFilterNode()
    try:
        node.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        rospy.loginfo("Exiting %s node" % node.node_name)
