#!/usr/bin/env python3
import os
import yaml
import math
from collections import OrderedDict 

import rospy
import actionlib
import dynamic_reconfigure.client

import tf2_ros
import tf_conversions
import tf2_geometry_msgs
from tf.transformations import euler_from_quaternion, quaternion_from_euler

from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Point
from geometry_msgs.msg import Vector3

from std_msgs.msg import ColorRGBA

from std_srvs.srv import Trigger, TriggerResponse

from visualization_msgs.msg import MarkerArray
from visualization_msgs.msg import Marker

from move_base_msgs.msg import MoveBaseAction

from vision_msgs.msg import Detection3DArray

from db_waypoints.srv import GetAllWaypoints, GetAllWaypointsResponse
from db_waypoints.srv import GetWaypoint, GetWaypointResponse
from db_waypoints.srv import DeleteWaypoint, DeleteWaypointResponse
from db_waypoints.srv import SavePose, SavePoseResponse
from db_waypoints.srv import SaveRobotPose, SaveRobotPoseResponse
from db_waypoints.srv import SaveTF, SaveTFResponse

from db_waypoints.msg import FollowPathAction, FollowPathGoal, FollowPathResult
from db_waypoints.msg import Waypoint, WaypointArray

from db_pursuit.msg import PursueObjectAction

from state_machine import WaypointStateMachine

from db_tools.yolo.utils import get_label, read_class_names



class SimpleDynamicToggle:
    def __init__(self, topic_name, config_key="enabled", **kwargs):
        self.topic_name = topic_name
        self.enabled = len(self.topic_name) > 0
        rospy.loginfo("Dynamic toggle (%s) enabled: %s" % (self.topic_name, self.enabled))
        
        self.state = None
        if self.enabled:
            self.dyn_client = dynamic_reconfigure.client.Client(
                self.topic_name,
                config_callback=self.callback,
                **kwargs
            )
        else:
            self.dyn_client = None
        self.config_key = config_key

    def callback(self, config):
        if not self.enabled:
            return 
        self.state = config[self.config_key]
        rospy.loginfo("Dynamic toggle (%s) callback: %s" % (self.topic_name, self.state))
    
    def set_state(self, state):
        if not self.enabled:
            return False
        if state == self.state:
            return False
        self.dyn_client.update_configuration({self.config_key: bool(state)})
        return True

    def get_state(self):
        return self.state


class DodobotWaypoints:
    def __init__(self):
        self.node_name = "db_waypoints"
        rospy.init_node(
            self.node_name
            # disable_signals=True
            # log_level=rospy.DEBUG
        )
        waypoints_path_param = rospy.get_param("~waypoints_path", "~/.ros/waypoints")
        waypoints_path_param = os.path.expanduser(waypoints_path_param)
        waypoints_path_param += ".yaml"
        self.class_names_path = rospy.get_param("~class_names_path", "objects.names")

        self.map_frame = rospy.get_param("~map", "map")
        self.base_frame = rospy.get_param("~base_link", "base_link")
        self.marker_size = rospy.get_param("~marker_size", 0.25)
        self.marker_color = rospy.get_param("~marker_color", (0.0, 0.0, 1.0, 1.0))
        self.enable_waypoint_navigation = rospy.get_param("~enable_waypoint_navigation", False)
        self.move_base_namespace = rospy.get_param("~move_base_namespace", "/move_base")
        self.pursuit_namespace = rospy.get_param("~pursuit_namespace", "/db/pursue_object")
        self.local_obstacle_layer_topic = rospy.get_param("~local_obstacle_layer_topic", "/move_base/local_costmap/obstacle_layer")
        self.global_obstacle_layer_topic = rospy.get_param("~global_obstacle_layer_topic", "/move_base/global_costmap/obstacle_layer")
        self.local_static_layer_topic = rospy.get_param("~local_static_layer_topic", "/move_base/local_costmap/static")
        self.global_static_layer_topic = rospy.get_param("~global_static_layer_topic", "/move_base/global_costmap/static")
        assert (type(self.marker_color) == tuple or type(self.marker_color) == list), "type(%s) != tuple or list" % type(self.marker_color)
        assert len(self.marker_color) == 4, "len(%s) != 4" % len(self.marker_color)

        self.waypoints_path = self.process_path(waypoints_path_param)
        self.waypoint_config = OrderedDict()

        self.class_names = read_class_names(self.class_names_path)

        self.nearest_objects = {}  # keys: object class names. values: [nearest PoseStamped, nearest distance (meters)]

        self.markers = MarkerArray()
        self.marker_poses = OrderedDict()

        self.load_from_path()  # load waypoints

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        if self.enable_waypoint_navigation:
            self.state_machine = WaypointStateMachine()
        else:
            self.state_machine = None

        self.marker_pub = rospy.Publisher("waypoint_markers", MarkerArray, queue_size=25)
        self.waypoints_pub = rospy.Publisher("waypoints", WaypointArray, queue_size=25)

        # self.detections_sub = rospy.Subscriber("detections", Detection3DArray, self.detection_callback)
        # self.cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)

        self.reload_waypoints_srv = self.create_service("reload_waypoints", Trigger, self.reload_waypoints_callback)
        self.get_all_waypoints_srv = self.create_service("get_all_waypoints", GetAllWaypoints, self.get_all_waypoints_callback)
        self.get_waypoint_srv = self.create_service("get_waypoint", GetWaypoint, self.get_waypoint_callback)
        self.delete_waypoint_srv = self.create_service("delete_waypoint", DeleteWaypoint, self.delete_waypoint_callback)
        self.save_pose_srv = self.create_service("save_pose", SavePose, self.save_pose_callback)
        self.save_tf_srv = self.create_service("save_tf", SaveTF, self.save_tf_callback)
        self.save_robot_pose_srv = self.create_service("save_robot_pose", SaveRobotPose, self.save_robot_pose_callback)

        if self.enable_waypoint_navigation:
            self.move_base = actionlib.SimpleActionClient(self.move_base_namespace, MoveBaseAction)

            rospy.loginfo("Connecting to move_base...")
            self.move_base.wait_for_server()
            rospy.loginfo("move_base connected")

            self.pursuit_action = actionlib.SimpleActionClient(self.pursuit_namespace, PursueObjectAction)

            rospy.loginfo("Connecting to pursuit...")
            self.pursuit_action.wait_for_server()
            rospy.loginfo("pursuit connected")

            self.local_obstacle_layer_toggle = SimpleDynamicToggle(self.local_obstacle_layer_topic, timeout=5)
            self.global_obstacle_layer_toggle = SimpleDynamicToggle(self.global_obstacle_layer_topic, timeout=5)
            self.local_static_layer_toggle = SimpleDynamicToggle(self.local_static_layer_topic, timeout=5)
            self.global_static_layer_toggle = SimpleDynamicToggle(self.global_static_layer_topic, timeout=5)
            rospy.loginfo("obstacle layer configs connected")
        else:
            self.move_base = None
            self.local_obstacle_layer_dyn_client = None
            self.global_obstacle_layer_dyn_client = None

        self.follow_path_server = actionlib.SimpleActionServer("follow_path", FollowPathAction, self.follow_path_callback, auto_start=False)
        self.follow_path_server.start()

        rospy.loginfo("%s is ready" % self.node_name)

    # ---
    # Action callback
    # ---

    def follow_path_callback(self, goal):
        rospy.loginfo("Received waypoint plan")
        if not self.enable_waypoint_navigation:
            rospy.logwarn("Navigation isn't enabled for this node. Set the parameter enable_waypoint_navigation to True")
            return
        
        waypoint_plan = self.get_waypoint_plan(goal.waypoints)
        if len(waypoint_plan) <= 0:
            print("Plan has no waypoints!")
            self.follow_path_server.set_aborted()
            return
        self.state_machine.execute(waypoint_plan, self)
        rospy.loginfo("Waypoint plan complete")

    # ---
    # Detection callback
    # ---

    def detection_callback(self, msg):
        self.nearest_objects = self.get_nearest_detections(msg)

    def get_nearest_detections(self, detections_msg):
        nearest_objs = {}
        for detection in detections_msg.detections:
            label, index = get_label(self.class_names, detection.results[0].id)
            detection_pose = detection.results[0].pose.pose
            detection_dist = self.get_distance(detection_pose)
            if label not in nearest_objs:
                nearest_objs[label] = [None, None]
            if nearest_objs[label][1] is None or detection_dist < nearest_objs[label][1]:
                nearest_pose = PoseStamped()
                nearest_pose.pose = detection_pose
                nearest_pose.header = detection.header
                nearest_objs[label][0] = nearest_pose
                nearest_objs[label][1] = detection_dist
        return nearest_objs

    def get_distance(self, pose1, pose2=None):
        if pose2 is None:
            x = pose1.position.x
            y = pose1.position.y
        else:
            x1 = pose1.position.x
            y1 = pose1.position.y
            x2 = pose2.position.x
            y2 = pose2.position.y
            x = x2 - x1
            y = y2 - y1
        return math.sqrt(x * x + y * y)

    # ---
    # Waypoint getters
    # ---

    def get_waypoint_plan(self, waypoints: WaypointArray):
        full_plan = []

        if len(waypoints.waypoints) <= 0:
            return full_plan

        sub_plan = []

        def reset_sub_plan():
            nonlocal sub_plan
            sub_plan = []  # list of db_waypoints/Waypoints

        def append_to_sub_plan(waypoint):
            nonlocal sub_plan
            if not self.is_waypoint_valid(waypoint.name):
                return

            sub_plan.append(waypoint)

        def append_to_full_plan():
            nonlocal sub_plan
            if len(sub_plan) == 0:
                reset_sub_plan()
                return

            # insert pose array to send to move_base.
            # only first waypoint in continuous sequence matters for other parameters like
            # ignore_orientation and intermediate_tolerance
            full_plan.append(sub_plan)
            reset_sub_plan()
        
        reset_sub_plan()

        waypoint = None
        for waypoint in waypoints.waypoints:
            append_to_sub_plan(waypoint)
            if not waypoint.is_continuous:
                append_to_full_plan()

        if len(sub_plan) > 0:  # if the last waypoint is continuous, make it discontinuous
            append_to_full_plan()
        return full_plan

    def get_waypoint_pose(self, waypoint: Waypoint):
        name = waypoint.name
        if len(name) == 0:
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = self.map_frame
            pose_stamped.pose = waypoint.pose
            return pose_stamped
        elif self.is_waypoint(name):
            pose_2d = self.get_waypoint(name)
            pose = self.waypoint_to_pose(pose_2d)
            return pose
        else:
            rospy.logwarn("Waypoint name '%s' is not registered." % name)
            return None

    def get_waypoints_as_pose_array(self, waypoints: list):
        pose_array = PoseArray()
        for waypoint in waypoints:
            pose_stamped = self.get_waypoint_pose(waypoint)
            if pose_stamped is None:
                continue
            pose_array.header = pose_stamped.header
            pose_array.poses.append(pose_stamped.pose)
        return pose_array

    def is_waypoint_valid(self, name):
        return self.is_waypoint(name) or self.is_object(name)

    # ---
    # set move_base parameters
    # ---

    def toggle_obstacles(self, state):
        state = bool(state)
        rospy.loginfo(("Enabling" if state else "Disabling") + " obstacle layer")
        local_changed = self.local_obstacle_layer_toggle.set_state(state)
        global_changed = self.global_obstacle_layer_toggle.set_state(state)
        return local_changed or global_changed
    
    def toggle_walls(self, state):
        state = bool(state)
        rospy.loginfo(("Enabling" if state else "Disabling") + " static layer")
        local_changed = self.local_static_layer_toggle.set_state(state)
        global_changed = self.global_static_layer_toggle.set_state(state)
        return local_changed or global_changed

    # ---
    # Service callbacks
    # ---

    def get_all_waypoints_callback(self, req):
        waypoints = self.get_all_waypoints()
        names = self.get_all_names()
        pose_array = self.waypoints_to_pose_array(waypoints)
        return GetAllWaypointsResponse(pose_array, names)

    def get_waypoint_callback(self, req):
        if not self.is_waypoint(req.name):
            return False
        
        pose_2d = self.get_waypoint(req.name)
        pose = self.waypoint_to_pose(pose_2d)
        return GetWaypointResponse(pose)
    
    def delete_waypoint_callback(self, req):
        if not self.is_waypoint(req.name):
            return False

        success = self.pop_waypoint(req.name)
        return DeleteWaypointResponse(success)

    def save_pose_callback(self, req):
        success = self.save_from_pose(req.name, req.waypoint)
        return SavePoseResponse(success)

    def save_robot_pose_callback(self, req):
        success = self.save_from_current(req.name)
        return SaveRobotPoseResponse(success)

    def save_tf_callback(self, req):
        success = self.save_from_tf(req.name, req.frame)
        return SaveTFResponse(success)

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

    def process_path(self, waypoints_path):
        map_name = os.path.basename(waypoints_path)
        waypoints_dir = os.path.dirname(waypoints_path)
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

    def is_waypoint(self, name):
        if name not in self.waypoint_config:
            return False
        if name not in self.marker_poses:
            rospy.logwarn("Waypoint name %s was added, but wasn't a registered marker! Adding." % name)
            pose = self.waypoint_to_pose(self.get_waypoint(name))
            self.add_marker(name, pose)
        return True
    
    def is_object(self, name):
        return name in self.class_names

    def save_from_pose(self, name, pose):
        # name: str, name of waypoint
        # pose: PoseStamped
        # returns: bool, whether the file was successfully written to
        self.waypoint_config[name] = self.pose_to_waypoint(pose)
        self.add_marker(name, pose)
        return self.save_to_path()

    def save_from_current(self, name):
        # name: str, name of waypoint
        # returns: bool, whether the file was successfully written to and whether the tf lookup was successful
        return self.save_from_tf(name, self.base_frame)

    def save_from_tf(self, name, frame):
        # name: str, name of waypoint
        # returns: bool, whether the file was successfully written to and whether the tf lookup was successful
        try:
            current_tf = self.tf_buffer.lookup_transform(self.map_frame, frame, rospy.Time(0), rospy.Duration(1.0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn("Failed to look up %s to %s. %s" % (self.map_frame, frame, e))
            return False
        
        pose = PoseStamped()
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
        # pose: PoseStamped
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
        # returns: PoseStamped
        quat = quaternion_from_euler(0.0, 0.0, waypoint[2])
        pose = PoseStamped()
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
        # returns: PoseArray
        pose_array = PoseArray()
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
        
        p1 = Point()
        p2 = Point()
        
        p2.x = self.marker_size

        position_marker.points.append(p1)
        position_marker.points.append(p2)
    
    def make_marker(self, name, pose):
        # name: str, marker name
        # pose: PoseStamped
        marker = Marker()
        marker.action = Marker.ADD
        marker.pose = pose.pose
        marker.header.frame_id = self.map_frame
        marker.lifetime = rospy.Duration(1.0)  # seconds
        marker.ns = name
        marker.id = 0  # all waypoint names should be unique

        scale_vector = Vector3()
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

    def publish_waypoints(self):
        waypoint_array = WaypointArray()
        for name, waypoint in self.waypoint_config.items():
            pose = self.waypoint_to_pose(waypoint)
            waypoint_msg = Waypoint()
            waypoint_msg.pose = pose.pose
            waypoint_msg.name = name
            waypoint_array.waypoints.append(waypoint_msg)

        self.waypoints_pub.publish(waypoint_array)

    # ---
    # Run
    # ---

    def run(self):
        rate = rospy.Rate(3.0)
        while not rospy.is_shutdown():
            self.publish_markers()
            self.publish_waypoints()
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
