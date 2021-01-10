#!/usr/bin/env python
import os
import yaml
import math
from collections import OrderedDict 

import rospy

import tf2_ros
import tf_conversions
import tf2_geometry_msgs
from tf.transformations import euler_from_quaternion, quaternion_from_euler

import geometry_msgs

from std_msgs.msg import ColorRGBA

from std_srvs.srv import Trigger, TriggerResponse

from visualization_msgs.msg import MarkerArray
from visualization_msgs.msg import Marker

from db_waypoints.srv import GetAllWaypoints, GetAllWaypointsResponse
from db_waypoints.srv import GetWaypoint, GetWaypointResponse
from db_waypoints.srv import DeleteWaypoint, DeleteWaypointResponse
from db_waypoints.srv import SavePose, SavePoseResponse
from db_waypoints.srv import SaveRobotPose, SaveRobotPoseResponse


class DodobotWaypoints:
    def __init__(self):
        self.node_name = "db_waypoints"
        rospy.init_node(
            self.node_name
            # disable_signals=True
            # log_level=rospy.DEBUG
        )
        self.map_path = rospy.get_param("~map_path", "~/.ros/rtabmap.db")
        self.map_path = os.path.expanduser(self.map_path)
        
        self.map_frame = rospy.get_param("~map", "map")
        self.base_frame = rospy.get_param("~base_link", "base_link")
        self.marker_size = rospy.get_param("~marker_size", 0.25)
        self.marker_color = rospy.get_param("~marker_color", (0.0, 0.0, 1.0, 1.0))
        assert (type(self.marker_color) == tuple or type(self.marker_color) == list), "type(%s) != tuple or list" % type(self.marker_color)
        assert len(self.marker_color) == 4, "len(%s) != 4" % len(self.marker_color)
        
        self.waypoints_path = self.process_path(self.map_path)
        self.waypoint_config = OrderedDict()

        self.markers = MarkerArray()
        self.marker_poses = OrderedDict()

        self.load_from_path()  # load waypoints

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.reload_waypoints_srv = self.create_service("reload_waypoints", Trigger, self.reload_waypoints_callback)
        
        self.get_all_waypoints_srv = self.create_service("get_all_waypoints", GetAllWaypoints, self.get_all_waypoints_callback)
        self.get_waypoint_srv = self.create_service("get_waypoint", GetWaypoint, self.get_waypoint_callback)
        self.delete_waypoint_srv = self.create_service("delete_waypoint", DeleteWaypoint, self.delete_waypoint_callback)
        self.save_pose_srv = self.create_service("save_pose", SavePose, self.save_pose_callback)
        self.save_robot_pose_srv = self.create_service("save_robot_pose", SaveRobotPose, self.save_robot_pose_callback)
    
        self.marker_pub = rospy.Publisher("waypoint_markers", MarkerArray, queue_size=25)

    # ---
    # Service callbacks
    # ---

    def get_all_waypoints_callback(self, req):
        waypoints = self.get_all_waypoints()
        names = self.get_all_names()
        pose_array = self.waypoints_to_pose_array(waypoints)
        return GetAllWaypointsResponse(pose_array, names)

    def get_waypoint_callback(self, req):
        waypoint = self.get_waypoint(req.name)
        pose = self.waypoint_to_pose(waypoint)
        return GetWaypointResponse(pose)
    
    def delete_waypoint_callback(self, req):
        success = self.pop_waypoint(req.name)
        return DeleteWaypointResponse(success)

    def save_pose_callback(self, req):
        success = self.save_from_pose(req.name, req.waypoint)
        return SavePoseResponse(success)

    def save_robot_pose_callback(self, req):
        success = self.save_from_current(req.name)
        return SaveRobotPoseResponse(success)

    def reload_waypoints_callback(self, req):
        if self.load_from_path():
            return TriggerResponse(True, self.waypoints_path)
        else:
            return TriggerResponse(False, self.waypoints_path)

    # ---
    # Service creation macros
    # ---

    def create_service(self, name, srv_type, callback):
        name = self.node_name + "/" + name
        service_name = name + "_service_name"
        self.__dict__[service_name] = name
        rospy.loginfo("Setting up service %s" % name)

        srv_obj = rospy.Service(name, srv_type, callback)
        rospy.loginfo("%s service is ready" % name)
        return srv_obj
    
    def listen_for_service(self, name, srv_type):
        service_name = name + "_service_name"
        self.__dict__[service_name] = name
        rospy.loginfo("Waiting for service %s" % name)

        srv_obj = rospy.ServiceProxy(name, srv_type)
        rospy.loginfo("%s service is ready" % name)
        return srv_obj

    # ---
    # File manipulations
    # ---

    def load_from_path(self):
        if not self.initialize_file():
            return False
        try:
            with open(self.waypoints_path) as file:
                config = yaml.safe_load(file)
            if config is None:
                self.waypoint_config = OrderedDict()
            else:
                self.waypoint_config = config
            self.all_waypoints_to_markers()
            return True
        except BaseException as e:
            rospy.logwarn("Failed to load waypoints file '%s'. %s" % (self.waypoints_path, e))
            return False
    
    def initialize_file(self):
        # If file doesn't exist, create directories and empty file
        if os.path.isfile(self.waypoints_path):
            return True
        waypoints_dir = os.path.dirname(self.waypoints_path)
        if not os.path.isdir(waypoints_dir):
            os.makedirs(waypoints_dir)
        with open(self.waypoints_path, 'w') as file:
            file.write("")
        rospy.logwarn("Waypoints file '%s' doesn't exist. Creating file." % self.waypoints_path)
        return False

    def process_path(self, map_path):
        map_name = os.path.basename(map_path)
        waypoints_dir = os.path.dirname(map_path)
        if len(waypoints_dir) == 0:
            waypoints_dir = os.path.expanduser("~/.ros")
        waypoints_name = os.path.splitext(map_name)[0]
        waypoints_name += ".yaml"
        waypoints_path = os.path.join(waypoints_dir, waypoints_name)
        return waypoints_path
    
    def save_to_path(self):
        try:
            with open(self.waypoints_path, 'w') as file:
                for name, waypoint in self.waypoint_config.items():
                    yaml.safe_dump({name: waypoint}, file)
            return True
        except BaseException as e:
            rospy.logwarn("Failed to save waypoints file '%s'. %s" % (self.waypoints_path, e))
            return False
    
    # ---
    # Node methods
    # ---

    def save_from_pose(self, name, pose):
        # name: str, name of waypoint
        # pose: geometry_msgs.msg.PoseStamped
        # returns: bool, whether the file was successfully written to
        self.waypoint_config[name] = self.pose_to_waypoint(pose)
        self.add_marker(name, pose)
        return self.save_to_path()

    def save_from_current(self, name):
        # name: str, name of waypoint
        # returns: bool, whether the file was successfully written to and whether the tf lookup was successful
        try:
            current_tf = self.tf_buffer.lookup_transform(self.map_frame, self.base_frame, rospy.Time(0), rospy.Duration(1.0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn("Failed to look up %s to %s. %s" % (self.map_frame, self.base_frame, e))
            return False
        
        pose = geometry_msgs.msg.PoseStamped()
        pose.header.frame_id = self.map_frame
        pose.pose.position = current_tf.transform.translation
        pose.pose.orientation = current_tf.transform.rotation
        
        return self.save_from_pose(name, pose)

    def save_from_object(self, name):
        # name: str, name of waypoint
        return False

    def get_waypoint(self, name):
        # name: str, name of waypoint
        # returns: list, [x, y, theta]
        return self.waypoint_config[name]
    
    def pop_waypoint(self, name):
        # name: str, name of waypoint
        # returns: list, [x, y, theta]
        self.delete_marker(name)
        self.waypoint_config.pop(name)
        return self.save_to_path()

    def get_all_waypoints(self):
        # returns: list, [[x, y, theta], ...]
        return [waypoint for waypoint in self.waypoint_config.values()]
    
    def get_all_names(self):
        # returns: list, [str, ...] waypoint names
        return [name for name in self.waypoint_config.keys()]

    # ---
    # Conversion methods
    # ---

    def pose_to_waypoint(self, pose):
        # pose: geometry_msgs.msg.PoseStamped
        # returns: list, [x, y, theta]
        yaw = euler_from_quaternion([
            pose.pose.orientation.x,
            pose.pose.orientation.y,
            pose.pose.orientation.z,
            pose.pose.orientation.w,
        ])[2]
        return [pose.pose.position.x, pose.pose.position.y, yaw]

    def waypoint_to_pose(self, waypoint):
        # waypoint: list, [x, y, theta]
        # returns: geometry_msgs.msg.PoseStamped
        quat = quaternion_from_euler(0.0, 0.0, waypoint[2])
        pose = geometry_msgs.msg.PoseStamped()
        pose.header.frame_id = self.map_frame
        pose.pose.position.x = waypoint[0]
        pose.pose.position.y = waypoint[1]
        pose.pose.orientation.x = quat[0]
        pose.pose.orientation.y = quat[1]
        pose.pose.orientation.z = quat[2]
        pose.pose.orientation.w = quat[3]
        return pose

    def waypoints_to_pose_array(self, waypoints):
        # waypoint: list, [[x, y, theta], ...]
        # returns: geometry_msgs.msg.PoseArray
        pose_array = geometry_msgs.msg.PoseArray()
        pose_array.header.frame_id = self.map_frame
        for waypoint in waypoints:
            pose = self.waypoint_to_pose(waypoint)
            pose_array.poses.append(pose.pose)
        return pose_array

    # ---
    # Waypoint visualization
    # ---

    def all_waypoints_to_markers(self):
        self.marker_poses = OrderedDict()
        for name, waypoint in self.waypoint_config.items():
            self.marker_poses[name] = self.waypoint_to_pose(waypoint)
        self.update_markers()

    def add_marker(self, name, pose):
        self.marker_poses[name] = pose
        self.update_markers()
    
    def delete_marker(self, name):
        self.marker_poses.pop(name)
        self.update_markers()
    
    def update_markers(self):
        self.markers = MarkerArray()
        for name, pose in self.marker_poses.items():
            position_marker = self.make_marker(name, pose)
            text_marker = self.make_marker(name, pose)
            
            self.prep_position_marker(position_marker)
            
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.ns = "text" + text_marker.ns
            text_marker.text = name
            text_marker.scale.x = 0.0
            text_marker.scale.y = 0.0

            self.markers.markers.append(position_marker)
            self.markers.markers.append(text_marker)
    
    def prep_position_marker(self, position_marker):
        position_marker.type = Marker.ARROW
        position_marker.ns = "pos" + position_marker.ns
        position_marker.color.a = 0.75
        position_marker.scale.x = self.marker_size / 4.0
        position_marker.scale.y = self.marker_size / 2.5
        position_marker.scale.z = self.marker_size / 2.0
        
        p1 = geometry_msgs.msg.Point()
        p2 = geometry_msgs.msg.Point()
        
        p2.x = self.marker_size

        position_marker.points.append(p1)
        position_marker.points.append(p2)


    
    def make_marker(self, name, pose):
        # name: str, marker name
        # pose: geometry_msgs.msg.PoseStamped
        marker = Marker()
        marker.action = Marker.ADD
        marker.pose = pose.pose
        marker.header.frame_id = self.map_frame
        marker.lifetime = rospy.Duration(1.0)  # seconds
        marker.ns = name
        marker.id = 0  # all waypoint names should be unique

        scale_vector = geometry_msgs.msg.Vector3()
        scale_vector.x = self.marker_size
        scale_vector.y = self.marker_size
        scale_vector.z = self.marker_size
        marker.scale = scale_vector
        marker.color = ColorRGBA(
            r=self.marker_color[0],
            g=self.marker_color[1],
            b=self.marker_color[2],
            a=self.marker_color[3],
        )

        return marker

    def publish_markers(self):
        if len(self.markers.markers) != 0:
            self.marker_pub.publish(self.markers)

    # ---
    # Run
    # ---

    def run(self):
        rate = rospy.Rate(3.0)
        while not rospy.is_shutdown():
            self.publish_markers()
            rate.sleep()


def main():
    node = DodobotWaypoints()
    try:
        node.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        rospy.loginfo("Exiting %s node" % node.node_name)

if __name__ == "__main__":
    main()
