#!/usr/bin/env python3
import math
import yaml
import numpy as np

import rospy
import actionlib
from actionlib_msgs.msg import GoalStatus

import tf2_ros
import tf_conversions
import tf2_geometry_msgs
import tf.transformations

import dynamic_reconfigure.client

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseWithCovarianceStamped

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

from nav_msgs.srv import GetPlan

from std_srvs.srv import Trigger

from std_msgs.msg import Bool

from db_planning.msg import FrontLoaderAction, FrontLoaderGoal, FrontLoaderResult
from db_planning.msg import GripperAction, GripperGoal, GripperResult
from db_planning.msg import ObjectPursuitAction, ObjectPursuitGoal, ObjectPursuitResult

from db_planning.msg import SequenceRequestAction, SequenceRequestGoal, SequenceRequestResult

from db_planning.srv import GrabbingSrv, GrabbingSrvResponse

from db_planning.sequence_state_machine import SequenceStateMachine

from db_parsing.msg import DodobotLinear

from db_parsing.srv import DodobotGetState
from db_parsing.srv import DodobotSetState

# from db_audio.srv import PlayAudio
# from db_audio.srv import StopAudio

from db_waypoints.srv import GetAllWaypoints

from db_laser_slam.srv import SetSlamMode
from db_laser_slam.srv import GetSlamMode

from db_launcher.srv import SetLaunch
from db_launcher.srv import GetLaunch

from dodobot_tools.helpers import get_msg_properties
from dodobot_tools.robot_state import Pose2d


class CentralPlanning:
    """
    Class definition for central_planning ROS node.
    """

    def __init__(self):
        """
        Initializes ROS and necessary services and publishers.
        """

        self.node_name = "central_planning"
        rospy.init_node(
            self.node_name
            # disable_signals=True
            # log_level=rospy.DEBUG
        )
        rospy.on_shutdown(self.cancel)

        self.chassis_action_name = rospy.get_param("~chassis_action_name", "chassis_actions")
        self.front_loader_action_name = rospy.get_param("~front_loader_action_name", "front_loader_actions")
        self.gripper_action_name = rospy.get_param("~gripper_action_name", "gripper_actions")
        self.pursuit_namespace = rospy.get_param("~pursuit_namespace", "pursuit_actions")
        self.move_base_namespace = rospy.get_param("~move_base_namespace", "/move_base")

        self.map_frame = rospy.get_param("~global_frame", "map")
        self.base_link_frame = rospy.get_param("~robot_frame", "base_link")
        self.gripper_frame = rospy.get_param("~gripper_frame", "gripper_link")
        self.tilt_base_frame = rospy.get_param("~tilt_base_frame", "tilt_base_link")
        self.linear_frame = rospy.get_param("~linear_frame", "linear_link")

        self.charge_dock_frame = rospy.get_param("~charge_dock_frame", "charge_dock_target")

        self.set_motor_state_allowed = rospy.get_param("~set_motor_state_allowed", False)
        self.home_linear_allowed = rospy.get_param("~home_linear_allowed", False)
        self.set_navigation_allowed = rospy.get_param("~set_navigation_allowed", False)

        self.gripper_max_dist = rospy.get_param("~gripper_max_dist", 0.112)
        self.max_vel_x = rospy.get_param("~max_vel_x", 0.15)
        self.max_vel_theta = rospy.get_param("~max_vel_theta", 1.0)

        self.goal_distance_offset = rospy.get_param("~goal_distance_offset", 0.1)
        self.near_object_distance = rospy.get_param("~near_object_distance", 1.0)
        self.replan_distance = rospy.get_param("~replan_distance", 0.45)
        
        self.pickup_z_offset = rospy.get_param("~pickup_z_offset", 0.02)
        self.deliver_z_offset = rospy.get_param("~deliver_z_offset", 0.02)

        self.transport_z_height = rospy.get_param("~transport_z_height", 0.08)

        self.fast_stepper_speed = rospy.get_param("~fast_stepper_speed", 0.044)
        self.slow_stepper_speed = rospy.get_param("~slow_stepper_speed", 0.02)

        self.plow_into_object_offset = rospy.get_param("~plow_into_object_offset", 0.025)

        self.costmap_size = rospy.get_param("~costmap_size", 3.0)
        self.costmap_box_radius = self.costmap_size / 2.0

        self.local_costmap_enabled_state = None

        self.is_charging = None
        
        self.default_max_vel = 0.0
        self.default_max_vel_theta = 0.0

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.valid_action_types = (
            SequenceRequestGoal.PICKUP,
            SequenceRequestGoal.DELIVER,
            SequenceRequestGoal.PARK,
            SequenceRequestGoal.DOCK,
            SequenceRequestGoal.UNDOCK
        )
        self.valid_goal_types = SequenceRequestGoal.NAMED_GOAL, SequenceRequestGoal.POSE_GOAL

        self.sequence_sm = SequenceStateMachine(self)

        # are robot motors enabled service
        self.get_robot_state = self.make_service_client("get_state", DodobotGetState)

        # set motors enabled service
        self.set_robot_state = self.make_service_client("set_state", DodobotSetState)

        if self.set_motor_state_allowed:
            self.set_drive_motors_active(True)

        # front loader action client
        self.front_loader_action = self.make_action_client(self.front_loader_action_name, FrontLoaderAction)

        # gripper action client
        self.gripper_action = self.make_action_client(self.gripper_action_name, GripperAction)

        # front loader ready service
        self.front_loader_ready_srv = self.make_service_client("front_loader_ready_service", Trigger)

        # linear commands topic (for homing)
        self.linear_pub = rospy.Publisher("linear_cmd", DodobotLinear, queue_size=5)
        
        # pursuit action client
        self.pursuit_client = self.make_action_client(self.pursuit_namespace, ObjectPursuitAction)
        
        # is gripper grabbing service
        self.gripper_grabbing_srv = self.make_service_client("gripper_grabbing_service", GrabbingSrv)

        # move_base action client
        self.move_action_client = self.make_action_client(self.move_base_namespace, MoveBaseAction, wait=False)
        
        # move_base make_plan service
        self.make_plan_srv = self.make_service_client(self.move_base_namespace + "/make_plan", GetPlan, wait=False)
        
        # get all waypoints service
        self.get_all_waypoints_srv = self.make_service_client("db_waypoints/get_all_waypoints", GetAllWaypoints)
        
        # obstacle layer dynamic reconfigure
        self.obstacle_layer_dyn_client = dynamic_reconfigure.client.Client("/move_base/local_costmap/obstacle_layer", timeout=30, config_callback=self.dyn_obstacle_layer_callback)

        # local planner dynamic reconfigure
        self.local_planner_name = "TebLocalPlannerROS"
        self.local_planner_dyn_client = dynamic_reconfigure.client.Client("/move_base/%s" % self.local_planner_name, timeout=30, config_callback=self.dyn_local_planner_callback)
        self.local_planner_start_config = self.local_planner_dyn_client.get_configuration()

        # camera launcher services
        self.start_camera_srv = self.make_service_client("start_camera", Trigger)
        self.stop_camera_srv = self.make_service_client("stop_camera", Trigger)
        self.is_camera_running_srv = self.make_service_client("is_camera_running", Trigger)

        # laser slam launcher services
        self.set_slam_mode_srv = self.make_service_client("set_slam_mode", SetSlamMode)
        self.get_slam_mode_srv = self.make_service_client("get_slam_mode", GetSlamMode)

        # general launcher service
        self.set_launch_srv = self.make_service_client("set_launch", SetLaunch)
        self.get_launch_srv = self.make_service_client("get_launch", GetLaunch)

        # camera tilter topic
        self.tilter_pub = rospy.Publisher("tilter_orientation", Quaternion, queue_size=100)

        # pursuit goal updater topic
        self.pursuit_goal_pub = rospy.Publisher("pursuit_goal", PoseStamped, queue_size=10)

        # direct velocity command topic
        self.cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=100)

        # is charging topic
        self.linear_sub = rospy.Subscriber("is_charging", Bool, self.is_charging_callback, queue_size=10)

        self.pose_estimate_pub = rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size=5)

        # audio services
        # self.play_audio_srv = self.make_service_client("play_audio", PlayAudio, wait=False)
        # self.stop_audio_srv = self.make_service_client("stop_audio", StopAudio, wait=False)

        # central_planning sequence action server
        self.sequence_sm.run_server()
        rospy.loginfo("[%s] Dodobot sequence server started" % self.node_name)

        while self.is_charging is None:
            rospy.sleep(0.1)
        rospy.loginfo("Charging signal received")

        rospy.loginfo("[%s] --- Dodobot central planning is up! ---" % self.node_name)
    
    def make_action_client(self, name, action, wait=True):
        """
        Create an actionlib client and wait for the server to connect
        """
        action = actionlib.SimpleActionClient(name, action)
        if wait:
            rospy.loginfo("Waiting for action server %s" % name)
            action.wait_for_server()
            rospy.loginfo("[%s] %s action server connected" % (self.node_name, name))
        return action
    
    def make_service_client(self, name, srv_type, timeout=None, wait=True):
        """
        Create a ros service client. Optionally wait with or without a timeout for the server to connect
        """
        self.__dict__[name + "_service_name"] = name
        rospy.loginfo("Connecting to %s service" % name)
        srv_obj = rospy.ServiceProxy(name, srv_type)

        if wait:
            rospy.loginfo("Waiting for service %s" % name)
            rospy.wait_for_service(name, timeout=timeout)
            rospy.loginfo("%s service is ready" % name)
        return srv_obj
    
    def dyn_obstacle_layer_callback(self, config):
        rospy.logdebug("Obstacle layer config set to %s" % str(config))
    
    def dyn_local_planner_callback(self, config):
        rospy.logdebug("Local planner config set to %s" % str(config))

    def is_charging_callback(self, msg):
        self.is_charging = msg.data

    def is_gripper_ok(self, goal):
        """
        Return true if the action sequence is PICKUP and if the gripper is empty.
        Return true if the action sequence is DELIVER and if the gripper is grabbing.
        Return true if the action sequence is PARK and if the gripper is empty.
        """
        is_grabbing = self.is_gripper_grabbing()

        if goal.action == SequenceRequestGoal.PICKUP and not is_grabbing:
            return True
        elif goal.action == SequenceRequestGoal.DELIVER and is_grabbing:
            return True
        elif goal.action == SequenceRequestGoal.PARK or goal.action == SequenceRequestGoal.UNDOCK:
            return True
        elif goal.action == SequenceRequestGoal.DOCK and not is_grabbing:
            return True
        else:
            return False
    
    def lookup_waypoint(self, name):
        response = self.get_all_waypoints_srv()
        for index, waypoint_name in enumerate(response.names):
            if name == waypoint_name:
                pose = response.waypoints.poses[index]
                pose_stamped = PoseStamped()
                pose_stamped.header = response.waypoints.header
                pose_stamped.pose = pose
                return pose_stamped
        return None

    def does_goal_exist(self, goal):
        """
        Check if the goal exists in the TF tree.
        If it's a pose, check if its header is in the TF tree.
        If it's a name, check if the name exists in the TF tree.
        """
        # check if map frame exists
        transform = self.lookup_transform(self.map_frame, self.base_link_frame)
        if transform is None:
            return None
        
        return self.get_goal_pose(goal) is not None

    def get_goal_pose(self, goal, parent_frame=None):
        """
        Get the goal as a PoseStamped object in the parent_frame.
        If parent_frame is None, use map_frame
        """
        if parent_frame is None:
            parent_frame = self.map_frame

        if goal.type == SequenceRequestGoal.POSE_GOAL:
            goal_frame = goal.pose_stamped.header.frame_id
            if goal_frame == parent_frame:
                return goal.pose_stamped
            else:
                transform = self.lookup_transform(parent_frame, goal_frame)
                if transform is None:
                    return None
                pose_map_frame = tf2_geometry_msgs.do_transform_pose(goal.pose_stamped, transform)
                return pose_map_frame

        elif goal.type == SequenceRequestGoal.NAMED_GOAL:
            return self.get_pose_in_frame(parent_frame, goal.name)
        
        rospy.logwarn("Incorrect goal_type. This should have been checked in the precheck state!")
        return None
    
    def get_pose_in_frame(self, parent_frame, child_frame, pose=None):
        """
        Get TF between parent and child frame as a PoseStamped object
        """
        waypoint = self.lookup_waypoint(child_frame)
        if waypoint is not None:
            rospy.loginfo("%s is a waypoint. Returning look up relative to waypoint: %s" % (child_frame, waypoint))
            return waypoint

        transform = self.lookup_transform(parent_frame, child_frame)
        if transform is None:
            return None
        if pose is None:
            zero_pose = PoseStamped()
            zero_pose.header.frame_id = child_frame
            zero_pose.pose.orientation.w = 1.0
            pose = zero_pose
        pose_map_frame = tf2_geometry_msgs.do_transform_pose(pose, transform)
        rospy.loginfo("%s -> %s is a TF. Returning look up relative to TF: %s" % (parent_frame, child_frame, pose_map_frame))
        return pose_map_frame

    def get_robot_pose(self):
        """
        Get robot's pose (TF between map and base_link)
        """
        return self.get_pose_in_frame(self.map_frame, self.base_link_frame)
    
    def get_nav_goal(self, goal):
        """
        Compute a pose some distance away from the goal 
        for navigation planners such that the gripper_link is above the object in X, Y
        """
        goal_pose = self.get_path_at_distance(goal, self.goal_distance_offset)
        if goal_pose is not None:
            goal_pose = self.offset_with_gripper(goal_pose)
        return goal_pose
    
    def average_within_std_dev(self, samples, std_dev_threshold):
        avg = np.average(samples, axis=0)
        std = np.std(samples, axis=0)
        std_bounds = std * std_dev_threshold
        centered = samples - avg
        out_bounds_indices = []
        for index in range(len(centered)):
            centered_sample = centered[index]
            if not np.all((-std_bounds <= centered_sample) & (centered_sample <= std_bounds)):
                out_bounds_indices.append(index)
        culled_samples = np.delete(samples, out_bounds_indices, axis=0)
        selected_sample = np.average(culled_samples, axis=0)
        
        return selected_sample
    
    def get_charge_dock_goal(self, std_dev_threshold=1.0, num_samples=10):
        samples = []
        for _ in range(num_samples):
            pose = self.get_pose_in_frame(self.map_frame, self.charge_dock_frame)
            pose_2d = Pose2d.from_ros_pose(pose.pose)
            samples.append(pose_2d)
        samples = Pose2d.to_array(samples)
        
        selected_sample = self.average_within_std_dev(samples, std_dev_threshold)

        selected_pose_2d = Pose2d.from_xyt(*selected_sample.tolist())
        selected_pose_3d = selected_pose_2d.to_ros_pose()

        dock_goal = PoseStamped()
        dock_goal.header.frame_id = self.map_frame
        dock_goal.pose = selected_pose_3d

        return dock_goal

    def get_goal_pose_with_gripper(self, goal):
        """
        Compute a pose for navigation planners such that the gripper_link is above the object in X, Y
        """
        goal_pose = self.get_goal_pose(goal)
        if goal_pose is not None:
            goal_pose = self.offset_with_gripper(goal_pose)
        return goal_pose
    
    def get_goal_with_orientation(self, goal, orientation):
        """
        Use the orientation result from 'get_nav_goal' or 'get_goal_pose_with_gripper' and apply it to the goal.
        This is for situations where the object moves to a new location part way through getting the robot there.
        """
        goal_pose = self.get_goal_pose(goal)
        if goal_pose is None:
            rospy.logwarn("Failed to get path to goal. Goal pose is not valid")
            return None
        # TODO: verify that this actually works as expected
        
        goal_pose.pose.orientation = orientation

        if goal_pose is not None:
            goal_pose = self.offset_with_gripper(goal_pose)
        return goal_pose
    
    def set_move_base_goal(self, goal_pose):
        """
        Send a PoseStamped object to the move_base action client.
        Tell the robot to go to a pose
        """
        goal = MoveBaseGoal()
        goal.target_pose = goal_pose
        self.move_action_client.send_goal(goal)
    
    def get_move_base_state(self):
        """
        Return move_base's state.
        Can be SUCCEEDED, ABORTED, ACTIVE (in the actionlib_msgs.msg.GoalStatus class)
        """
        return self.move_action_client.get_state()

    def get_path_at_distance(self, goal, distance):
        """
        Get a pose at a radial distance along move_base's computed path to the goal.
        If all poses are less than the distance threshold, use the starting pose
        """
        start_pose = self.get_pose_in_frame(self.map_frame, self.base_link_frame)
        goal_pose = self.get_goal_pose(goal)
        if goal_pose is None:
            rospy.logwarn("Failed to get path to goal. Goal pose is not valid")
            return None

        resp = self.make_plan_srv(start=start_pose, goal=goal_pose, tolerance=self.near_object_distance)
        
        if resp and len(resp.plan.poses) > 0:
            if distance == 0.0:
                final_pose = resp.plan.poses[-1]
            else:
                path_index = len(resp.plan.poses) - 1
                while self.get_pose_distance(start_pose, resp.plan.poses[path_index]) > distance:
                    path_index -= 1
                    if path_index < 0:
                        rospy.logwarn("All poses in the plan are less than the distance threshold '%s'. "
                                    "Using starting pose." % distance)
                        path_index = 0
                        break
                final_pose = resp.plan.poses[path_index]
            
            # compute the heading between the distanced final pose and the goal pose
            final_pose_2d = Pose2d.from_ros_pose(final_pose.pose)
            goal_pose_2d = Pose2d.from_ros_pose(goal_pose.pose)

            # assign that heading as the final goal orientation
            final_heading = Pose2d(theta=goal_pose_2d.heading(final_pose_2d))
            goal_pose.pose.orientation = final_heading.get_theta_as_quat()
            return goal_pose
        else:
            rospy.logwarn("Failed to get path to goal. move_base failed to produce a plan")
            return None
    
    def get_pose_distance(self, pose_stamped1, pose_stamped2):
        """
        Compute the distance between two PoseStamped objects.
        Assuming both objects are in the same frame
        """
        assert pose_stamped1.header.frame_id == pose_stamped2.header.frame_id, "%s != %s" % (
            pose_stamped1.header.frame_id, pose_stamped2.header.frame_id
        )
        point1 = np.array([
            pose_stamped1.pose.position.x,
            pose_stamped1.pose.position.y,
            pose_stamped1.pose.position.z
        ])
        point2 = np.array([
            pose_stamped2.pose.position.x,
            pose_stamped2.pose.position.y,
            pose_stamped2.pose.position.z
        ])
        distance = np.linalg.norm(point2 - point1)
        return distance

    def offset_with_gripper(self, pose_stamped) -> PoseStamped:
        """
        Offset the pose_stamped object by the offset between base_link and gripper_link
        """
        gripper_to_base_link = self.lookup_transform(self.gripper_frame, self.base_link_frame)
        gripper_to_goal = self.lookup_transform(pose_stamped.header.frame_id, self.gripper_frame)
        goal_to_gripper = self.lookup_transform(self.gripper_frame, pose_stamped.header.frame_id)
        if goal_to_gripper is None or gripper_to_base_link is None or gripper_to_goal is None:
            return None

        # the goal pose in the gripper frame
        pose_gripper = tf2_geometry_msgs.do_transform_pose(pose_stamped, goal_to_gripper)

        # offset the goal in the gripper frame by the offset between base_link and gripper_link
        pose_gripper.pose.position.x += gripper_to_base_link.transform.translation.x
        pose_gripper.pose.position.y += gripper_to_base_link.transform.translation.y
        pose_gripper.pose.position.z += gripper_to_base_link.transform.translation.z

        pose_gripper.pose.position.x += self.plow_into_object_offset

        # rotate the goal in the gripper frame by the rotation between base_link and gripper_link (should be no change)
        pose_gripper.pose.orientation = self.list_to_quat(tf.transformations.quaternion_multiply(
            self.quat_to_list(pose_gripper.pose.orientation),
            self.quat_to_list(gripper_to_base_link.transform.rotation)
        ))

        # transform goal pose from gripper_link back to the global frame
        pose_stamped_gripper_offset = tf2_geometry_msgs.do_transform_pose(pose_gripper, gripper_to_goal)
        
        return pose_stamped_gripper_offset
    
    def quat_to_list(self, quat):
        return [quat.x, quat.y, quat.z, quat.w]
    
    def list_to_quat(self, quat_list):
        quat = Quaternion()
        quat.x = quat_list[0]
        quat.y = quat_list[1]
        quat.z = quat_list[2]
        quat.w = quat_list[3]
        return quat
    
    def rotate_quat(self, quat, angle_rad):
        euler = list(tf.transformations.euler_from_quaternion(self.quat_to_list(quat)))
        euler[2] += angle_rad
        return self.list_to_quat(tf.transformations.quaternion_from_euler(*euler))

    def get_euler_z(self, quat):
        euler = list(tf.transformations.euler_from_quaternion(self.quat_to_list(quat)))
        return euler[2]

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
    
    def look_at_goal(self, goal):
        """
        Point the camera at the goal.
        Compute the TF from the goal pose/name to the camera and calculate the angle.
        """
        if goal.type == SequenceRequestGoal.POSE_GOAL:
            
            # TODO: verify that this actually works as expected

            camera_base_tf = self.lookup_transform(self.map_frame, self.tilt_base_frame)
            if camera_base_tf is None:
                return
            pose_tilt_base_frame = tf2_geometry_msgs.do_transform_pose(goal.pose_stamped, camera_base_tf)
            object_x = pose_tilt_base_frame.pose.position.x
            object_z = pose_tilt_base_frame.pose.position.z
        elif goal.type == SequenceRequestGoal.NAMED_GOAL:
            waypoint = self.lookup_waypoint(goal.name)
            if waypoint is None:
                camera_base_tf = self.lookup_transform(self.tilt_base_frame, goal.name)
                if camera_base_tf is None:
                    return
                object_x = camera_base_tf.transform.translation.x
                object_z = camera_base_tf.transform.translation.z
            else:
                camera_base_tf = self.lookup_transform(self.map_frame, self.tilt_base_frame)
                if camera_base_tf is None:
                    return
                pose_tilt_base_frame = tf2_geometry_msgs.do_transform_pose(waypoint, camera_base_tf)
                object_x = pose_tilt_base_frame.pose.position.x
                object_z = pose_tilt_base_frame.pose.position.z
        else:
            return

        angle = math.atan2(object_z, object_x)
        self.tilt_camera(angle)
    
    def look_straight_ahead(self):
        """
        Point the camera horizontally (0 degrees)
        """
        self.tilt_camera(0.0)

    def tilt_camera(self, angle):
        """
        Set the camera to an angle (radians)
        """
        rospy.loginfo("Camera tilt angle (deg): %0.2f" % math.degrees(angle))

        orientation = Quaternion()
        quaternion = tf_conversions.transformations.quaternion_from_euler(0.0, angle, 0.0)
        orientation.x = quaternion[0]
        orientation.y = quaternion[1]
        orientation.z = quaternion[2]
        orientation.w = quaternion[3]

        self.tilter_pub.publish(orientation)
    
    def set_linear_z_to_transport(self, goal):
        if goal.action == SequenceRequestGoal.PICKUP:
            stepper_speed = self.fast_stepper_speed
        elif goal.action == SequenceRequestGoal.DELIVER:
            stepper_speed = self.slow_stepper_speed
        else:
            stepper_speed = float("nan")
        self.set_linear_z(self.transport_z_height, stepper_speed)
    
    def set_linear_z(self, z, speed=float("nan")):
        """
        Set linear stepper to a height z (meters)
        """
        rospy.loginfo("Central planning linear stepper command: z=%s, speed=%s" % (z, speed))
        front_loader_goal = FrontLoaderGoal()
        front_loader_goal.goal_z = z
        front_loader_goal.z_speed = speed
        front_loader_goal.z_accel = float("nan")
        self.front_loader_action.send_goal(front_loader_goal)
    
    def set_linear_z_to_object_height(self, goal_pose, goal, stepper_speed=float("nan")):
        self.set_linear_z(goal_pose.pose.position.z - self.pickup_z_offset + goal.object_z_offset, stepper_speed)

    def stop_motors(self):
        self.set_twist(0.0, 0.0)

    def set_twist(self, linear_x, angular_z):
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        self.cmd_vel_pub.publish(msg)

    def wait_for_linear_z(self):
        """
        Wait for the linear stepper to get to the specified height
        """
        self.front_loader_action.wait_for_result()
        front_loader_result = self.front_loader_action.get_result()
        return front_loader_result.success
    
    def close_gripper(self, distance=0.0, force_threshold=None):
        """
        Set gripper distance to zero with the default force threshold
        """
        gripper_goal = GripperGoal()
        gripper_goal.grip_distance = distance
        gripper_goal.force_threshold = float("nan") if force_threshold is None else force_threshold
        self.gripper_action.send_goal(gripper_goal)
    
    def open_gripper(self):
        """
        Set gripper distance to max_dist with the default force threshold
        """
        gripper_goal = GripperGoal()
        gripper_goal.grip_distance = self.gripper_max_dist
        gripper_goal.force_threshold = float("nan")  # not used for open gripper
        self.gripper_action.send_goal(gripper_goal)
    
    def wait_for_gripper(self):
        """
        Wait for the gripper to get to the specified distance
        """
        self.gripper_action.wait_for_result()
        gripper_result = self.gripper_action.get_result()
        return gripper_result.success
    
    def is_gripper_grabbing(self):
        result = self.gripper_grabbing_srv(-1)  # check with default gripping threshold
        return result.is_grabbing
    
    def toggle_local_costmap(self, state):
        state = bool(state)
        if state != self.local_costmap_enabled_state:
            self.local_costmap_enabled_state = state
        else:
            return
        rospy.loginfo("Changing obstacle layer state to %s" % state)
        self.obstacle_layer_dyn_client.update_configuration({"enabled": state})
        
    def set_planner_velocities(self, max_vel_x=None, max_vel_x_backwards=None, max_vel_theta=None):
        if max_vel_x is not None and max_vel_x_backwards is None:
            # max_vel_x_backwards = max(max_vel_x - 0.1, 0.0)
            max_vel_x_backwards = 0.0
        config = ""
        if self.local_planner_name == "TebLocalPlannerROS":
            # config += f"groups:\n"
            # config += f"  groups:\n"
            # config += f"    Robot:\n"
            config += f"max_vel_x: {max_vel_x}\n" if max_vel_x is not None else ""
            config += f"max_vel_x_backwards: {max_vel_x_backwards}\n" if max_vel_x_backwards is not None else ""
            config += f"max_vel_theta: {max_vel_theta}\n" if max_vel_theta is not None else ""
        self.set_planner(config)

    def set_planner(self, config_str):
        config = yaml.safe_load(config_str)
        rospy.loginfo("Supplied config: %s" % str(config))
        self.local_planner_dyn_client.update_configuration(config)
    
    def set_planner_to_default(self):
        self.local_planner_dyn_client.update_configuration(self.local_planner_start_config)

    def cancel_move_base(self):
        self.move_action_client.cancel_all_goals()
        if self.move_action_client.get_state() in (GoalStatus.ACTIVE, GoalStatus.PENDING, GoalStatus.PREEMPTING, GoalStatus.RECALLING):
            result = self.move_action_client.wait_for_result()
            rospy.loginfo("Canceled move_base: %s" % result)
    
    def init_pursuit_goal(self, goal_pose, **pursuit_parameters):
        parameters = get_msg_properties(ObjectPursuitGoal())
        for name, value in parameters.items():
            if name in pursuit_parameters:
                continue
            if type(value) == float:
                pursuit_parameters[name] = float("nan")
            if type(value) == bool:
                pursuit_parameters[name] = False

        goal = ObjectPursuitGoal(**pursuit_parameters)
        goal.pose = goal_pose
        self.pursuit_client.send_goal(goal)
    
    def set_pursuit_goal(self, goal_pose: PoseStamped):
        self.pursuit_goal_pub.publish(goal_pose)

    def get_pursuit_state(self):
        state = self.pursuit_client.get_state()
        if state == GoalStatus.ACTIVE:
            return "active"
        elif state == GoalStatus.PREEMPTED:
            return "preempted"
        elif state == GoalStatus.ABORTED:
            return "failure"
        elif state == GoalStatus.SUCCEEDED:
            return "success"
        else:
            return "??"

    def turn_in_place(self, angle):
        goal_pose = self.get_robot_pose()
        goal_pose.pose.orientation = self.rotate_quat(goal_pose.pose.orientation, angle)
        rospy.loginfo("Pursuit goal: %0.4f, %0.4f, %0.4f" % (
            goal_pose.pose.position.x,
            goal_pose.pose.position.y,
            self.get_euler_z(goal_pose.pose.orientation))
        )

        self.init_pursuit_goal(
            goal_pose,
            angle_tolerance=0.07,
            position_tolerance=1.0,
            timeout_fudge=1.0,
            timeout_turn_fudge=10.0,
            max_linear_speed=0.15,
            max_angular_speed=2.0,
            loopback_y_tolerance=1.0,
            turn_in_place_only=True,
        )

        check_state_interval = 0.1

        while True:
            if rospy.is_shutdown():
                return "preempted"

            rospy.sleep(check_state_interval)

            state = self.get_pursuit_state()
            if state == "success" or state == "failure" or state == "preempted":
                return state

    def drive_straight(self, distance_m):
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = self.base_link_frame
        goal_pose.pose.orientation.w = 1.0
        goal_pose.pose.position.x = distance_m
        goal_pose = self.get_pose_in_frame(self.map_frame, self.base_link_frame, goal_pose)

        rospy.loginfo("Pursuit goal: %0.4f, %0.4f, %0.4f" % (
            goal_pose.pose.position.x,
            goal_pose.pose.position.y,
            self.get_euler_z(goal_pose.pose.orientation))
        )

        self.init_pursuit_goal(
            goal_pose,
            angle_tolerance=0.15,
            position_tolerance=0.07,
            timeout_fudge=10.0,
            timeout_turn_fudge=1.0,
            max_linear_speed=0.5,
            max_angular_speed=2.0,
            loopback_y_tolerance=1.0,
            reversed=distance_m < 0.0
        )

        check_state_interval = 0.1

        while True:
            if rospy.is_shutdown():
                return "preempted"

            rospy.sleep(check_state_interval)

            state = self.get_pursuit_state()
            if state == "success" or state == "failure" or state == "preempted":
                return state

    def is_move_base_running(self):
        return self.get_launch_srv("move_base").is_running

    def cancel_pursuit_goal(self):
        self.pursuit_client.cancel_all_goals()
    
    def cancel(self):
        if self.is_move_base_running():
            self.toggle_local_costmap(True)
            self.cancel_move_base()
            self.set_planner_to_default()
        self.cancel_pursuit_goal()
        self.set_linear_z(float("nan"), self.fast_stepper_speed)

    def are_drive_motors_active(self):
        return self.get_robot_state().active

    def set_drive_motors_active(self, state):
        if self.set_motor_state_allowed:
            rospy.loginfo("Setting drive motors to %s" % "active" if state else "inactive")
            self.set_robot_state(True, bool(state))
        else:
            rospy.logwarn("Setting drive motor active is not permitted if set_motor_state_allowed is False")

    def get_linear_stepper_ready_state(self):
        return self.front_loader_ready_srv()

    def home_linear_stepper(self):
        if self.home_linear_allowed:
            rospy.loginfo("Homing linear stepper")
            msg = DodobotLinear()
            msg.command_type = 4  # home stepper command
            msg.max_speed = -1
            msg.acceleration = -1
            self.linear_pub.publish(msg)
        else:
            rospy.logwarn("Homing the linear stepper is not permitted if home_linear_allowed is False")

    def set_pose_estimate(self, pose_stamped, x_std=0.5, y_std=0.5, theta_std_deg=15.0):
        rospy.loginfo("Setting robot pose estimate to %s" % pose_stamped)
        pose_cov_stamped = PoseWithCovarianceStamped()
        pose_cov_stamped.header = pose_stamped.header
        pose_cov_stamped.pose.pose = pose_stamped.pose
        theta_std_rad = math.radians(theta_std_deg)
        pose_cov_stamped.pose.covariance = [
            x_std * x_std, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, y_std * y_std, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, theta_std_rad * theta_std_rad
        ]

        self.pose_estimate_pub.publish(pose_cov_stamped)

    def set_robot_pose_to_name(self, name, x_std=0.5, y_std=0.5, theta_std_deg=15.0):
        rospy.loginfo("Setting robot pose estimate to %s" % name)
        pose_stamped = self.get_pose_in_frame(self.map_frame, name)
        self.set_pose_estimate(pose_stamped, x_std, y_std, theta_std_deg)
    
    def is_robot_moving(self, distance_threshold_m=0.001, time_delay=0.05):
        pose1 = Pose2d.from_ros_pose(self.get_robot_pose().pose)
        rospy.sleep(time_delay)
        pose2 = Pose2d.from_ros_pose(self.get_robot_pose().pose)
        return pose1.distance(pose2) > distance_threshold_m

    def set_navigation_active(self):
        if not self.start_navigation_launches():
            rospy.logwarn("Navigation failed to start!")
            return False

        rospy.loginfo("Waiting for launches to start")
        if not self.wait_for_navigation_launches_to_start():
            rospy.logwarn("Timedout while waiting for navigation to start")
            return False

        rospy.loginfo("All launches started!")

        if self.is_charging:
            rospy.loginfo("Detected that robot is charging. Setting location to dock")
            self.set_robot_pose_to_name("dock")
        
        return True
    
    def set_navigation_idle(self):
        rospy.loginfo("Successfully docked. Shutting down navigation launches")
        if not self.stop_navigation_launches():
            rospy.logwarn("Navigation failed to stop!")
            return False

        rospy.loginfo("Waiting for launches to stop")
        if not self.wait_for_navigation_launches_to_stop():
            rospy.logwarn("Timedout while waiting for navigation to stop")
            return False
            
        rospy.loginfo("All launches stopped!")

        if self.set_motor_state_allowed:
            self.set_drive_motors_active(False)
            rospy.loginfo("Deactivating drive motors")
        return True

    def is_navigation_running(self):
        if not self.is_camera_running_srv().success:
            rospy.loginfo("Camera is not running")
            return False

        if self.get_slam_mode_srv().mode == "idle":
            rospy.loginfo("SLAM is not running")
            return False

        for launch_name in ("rplidar", "april_tags", "move_base"):
            if not self.get_launch_srv(launch_name).is_running:
                rospy.loginfo("%s is not running" % launch_name)
                return False
        
        return True

    def start_navigation_launches(self, slam_mode="localize", map_name=""):
        if not self.set_navigation_allowed:
            rospy.logerr("Setting navigation is not permitted if set_navigation_allowed is False")
            return False
        rospy.loginfo("Starting navigation launches")

        result = self.start_camera_srv()
        if not result.success:
            rospy.logerr(result.message)
            return False
        rospy.loginfo("Camera launch started")
        
        result = self.set_slam_mode_srv(slam_mode, map_name)
        if not result.success:
            rospy.logerr(result.message)
            return False
        rospy.loginfo("SLAM launch started")
        
        for launch_name in ("rplidar", "april_tags", "move_base"):
            result = self.set_launch_srv(launch_name, 1)  # SetLaunch.MODE_START
            if not result.success:
                rospy.logerr(result.message)
                return False
            rospy.loginfo("%s launch started" % launch_name)

        return True

    def stop_navigation_launches(self):
        if not self.set_navigation_allowed:
            rospy.logerr("Setting navigation is not permitted if set_navigation_allowed is False")
            return False
        
        result = self.stop_camera_srv()
        if not result.success:
            rospy.logerr(result.message)
            return False
        
        result = self.set_slam_mode_srv("idle", "")
        if not result.success:
            rospy.logerr(result.message)
            return False
        
        for launch_name in ("rplidar", "april_tags", "move_base"):
            result = self.set_launch_srv(launch_name, 2)  # SetLaunch.MODE_STOP
            if not result.success:
                rospy.logerr(result.message)
                return False

        return True
    
    def wait_for_navigation_launches_to_start(self, timeout_s=120.0):
        start_wait_time = rospy.Time.now()
        def has_timedout():
            return rospy.Time.now() - start_wait_time > rospy.Duration(timeout_s)

        while not self.is_camera_running_srv().success:
            rospy.sleep(0.1)
            if has_timedout():
                return False
        for launch_name in ("rplidar", "april_tags", "move_base"):
            while not self.get_launch_srv(launch_name).is_running:
                rospy.sleep(0.1)
                if has_timedout():
                    return False
        
        return True
    
    def wait_for_navigation_launches_to_stop(self, timeout_s=120.0):
        start_wait_time = rospy.Time.now()
        def has_timedout():
            return rospy.Time.now() - start_wait_time > rospy.Duration(timeout_s)

        while self.is_camera_running_srv().success:
            rospy.sleep(0.1)
            if has_timedout():
                return False
        for launch_name in ("rplidar", "april_tags", "move_base"):
            while self.get_launch_srv(launch_name).is_running:
                rospy.sleep(0.1)
                if has_timedout():
                    return False
        
        return True
    
    def run(self):
        if not self.is_charging and not self.is_navigation_running():
            if self.set_navigation_allowed:
                self.set_navigation_active()

        rospy.spin()
    
    def test_linear(self):
        sequence = [0.04, 0.08, 0.12, 0.14, 0.12, 0.08, 0.04, 0.0]
        while True:
            for position in sequence:
                self.set_linear_z(position, self.fast_stepper_speed)
                if not self.wait_for_linear_z():
                    break

if __name__ == "__main__":
    try:
        node = CentralPlanning()
        node.run()
        # node.test_linear()
    except rospy.ROSInterruptException:
        pass
    finally:
        rospy.loginfo("Exiting central_planning node")
