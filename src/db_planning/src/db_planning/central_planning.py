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

from geometry_msgs.msg import PoseStamped, Quaternion

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

from nav_msgs.srv import GetPlan

from std_srvs.srv import Trigger, TriggerResponse

import std_msgs.msg

from db_planning.msg import ChassisAction, ChassisGoal, ChassisResult
from db_planning.msg import FrontLoaderAction, FrontLoaderGoal, FrontLoaderResult
from db_planning.msg import GripperAction, GripperGoal, GripperResult
from db_planning.msg import ObjectPursuitAction, ObjectPursuitGoal, ObjectPursuitResult

from db_planning.msg import SequenceRequestAction, SequenceRequestGoal, SequenceRequestResult

from db_planning.srv import GrabbingSrv, GrabbingSrvResponse

from db_planning.sequence_state_machine import SequenceStateMachine

from db_parsing.srv import DodobotGetState

from db_audio.srv import PlayAudio
from db_audio.srv import StopAudio


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

        self.gripper_max_dist = rospy.get_param("~gripper_max_dist", 0.112)
        self.max_vel_x = rospy.get_param("~max_vel_x", 0.15)
        self.max_vel_theta = rospy.get_param("~max_vel_theta", 1.0)

        self.goal_distance_offset = rospy.get_param("~goal_distance_offset", 0.1)
        self.near_object_distance = rospy.get_param("~near_object_distance", 3.0)
        self.replan_distance = rospy.get_param("~replan_distance", 0.45)
        
        self.pickup_z_offset = rospy.get_param("~pickup_z_offset", 0.02)
        self.deliver_z_offset = rospy.get_param("~deliver_z_offset", 0.02)

        self.transport_z_height = rospy.get_param("~transport_z_height", 0.08)

        self.fast_stepper_speed = rospy.get_param("~fast_stepper_speed", 0.044)
        self.slow_stepper_speed = rospy.get_param("~slow_stepper_speed", 0.02)

        self.plow_into_object_offset = rospy.get_param("~plow_into_object_offset", 0.025)

        self.local_costmap_enabled_state = None
        
        self.default_max_vel = 0.0
        self.default_max_vel_theta = 0.0

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.valid_action_types = SequenceRequestGoal.PICKUP, SequenceRequestGoal.DELIVER
        self.valid_goal_types = SequenceRequestGoal.NAMED_GOAL, SequenceRequestGoal.POSE_GOAL

        self.sequence_sm = SequenceStateMachine(self)

        # front loader action client
        self.front_loader_action = self.make_action_client(self.front_loader_action_name, FrontLoaderAction)

        # gripper action client
        self.gripper_action = self.make_action_client(self.gripper_action_name, GripperAction)

        # front loader ready service
        self.front_loader_ready_srv = self.make_service_client("front_loader_ready_service", Trigger)

        # pursuit action client
        self.pursuit_client = self.make_action_client(self.pursuit_namespace, ObjectPursuitAction)
        
        # is gripper grabbing service
        self.gripper_grabbing_srv = self.make_service_client("gripper_grabbing_service", GrabbingSrv)

        # move_base action client
        self.move_action_client = self.make_action_client(self.move_base_namespace, MoveBaseAction)
        
        # move_base make_plan service
        self.make_plan_srv = self.make_service_client(self.move_base_namespace + "/make_plan", GetPlan)
        
        # obstacle layer dynamic reconfigure
        self.obstacle_layer_dyn_client = dynamic_reconfigure.client.Client("/move_base/local_costmap/obstacle_layer", timeout=30, config_callback=self.dyn_obstacle_layer_callback)

        # local planner dynamic reconfigure
        self.local_planner_name = "TebLocalPlannerROS"
        self.local_planner_dyn_client = dynamic_reconfigure.client.Client("/move_base/%s" % self.local_planner_name, timeout=30, config_callback=self.dyn_local_planner_callback)
        self.local_planner_start_config = self.local_planner_dyn_client.get_configuration()

        # camera tilter service
        self.tilter_pub = rospy.Publisher("tilter_orientation", Quaternion, queue_size=100)

        # are robot motors enabled service
        self.get_robot_state = self.make_service_client("get_state", DodobotGetState)

        # audio services
        self.play_audio_srv = self.make_service_client("play_audio", PlayAudio, wait=False)
        self.stop_audio_srv = self.make_service_client("stop_audio", StopAudio, wait=False)

        # central_planning sequence action server
        self.sequence_sm.run_server()
        rospy.loginfo("[%s] Dodobot sequence server started" % self.node_name)

        rospy.loginfo("[%s] --- Dodobot central planning is up! ---" % self.node_name)
    
    def make_action_client(self, name, action):
        """
        Create an actionlib client and wait for the server to connect
        """
        action = actionlib.SimpleActionClient(name, action)
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
        rospy.loginfo("Obstacle layer config set to %s" % str(config))
    
    def dyn_local_planner_callback(self, config):
        rospy.loginfo("Local planner config set to %s" % str(config))

    def is_gripper_ok(self, goal):
        """
        Return true if the action sequence is PICKUP and if the gripper is empty.
        Return true if the action sequence is DELIVER and if the gripper is grabbing.
        """
        is_grabbing = self.is_gripper_grabbing()

        if goal.action == SequenceRequestGoal.PICKUP and not is_grabbing:
            return True
        elif goal.action == SequenceRequestGoal.DELIVER and is_grabbing:
            return True
        else:
            return False
    
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
    
    def get_pose_in_frame(self, parent_frame, child_frame):
        """
        Get TF between parent and child frame as a PoseStamped object
        """
        transform = self.lookup_transform(parent_frame, child_frame)
        if transform is None:
            return None
        zero_pose = PoseStamped()
        zero_pose.header.frame_id = child_frame
        pose_map_frame = tf2_geometry_msgs.do_transform_pose(zero_pose, transform)
        return pose_map_frame

    def get_robot_pose(self):
        """
        Get robot's pose (TF between map and base_link)
        """
        return self.get_pose_in_frame(self.map_frame, self.base_link_frame)
    
    def get_nav_goal(self, goal):
        """
        Compute a goal for navigation planners such that the gripper_link is above the object in X, Y
        """
        goal_pose = self.get_path_at_distance(goal, self.goal_distance_offset)
        if goal_pose is not None:
            goal_pose = self.offset_with_gripper(goal_pose)
        return goal_pose
    
    def get_goal_with_orientation(self, goal, orientation):
        """
        Use the orientation result from 'get_nav_goal' and apply it to the goal.
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

        resp = self.make_plan_srv(start=start_pose, goal=goal_pose, tolerance=0.0)
        
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
            
            goal_pose.pose.orientation = final_pose.pose.orientation  # copy just the orientation from the final pose
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

    def offset_with_gripper(self, pose_stamped):
        """
        Offset the pose_stamped object by the offset between base_link and gripper_link
        """
        goal_to_gripper = self.lookup_transform(self.gripper_frame, pose_stamped.header.frame_id)
        gripper_to_base_link = self.lookup_transform(self.gripper_frame, self.base_link_frame)
        gripper_to_goal = self.lookup_transform(pose_stamped.header.frame_id, self.gripper_frame)
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
            camera_base_tf = self.lookup_transform(self.tilt_base_frame, goal.name)
            if camera_base_tf is None:
                return
            object_x = camera_base_tf.transform.translation.x
            object_z = camera_base_tf.transform.translation.z
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
    
    def set_pursuit_goal(self, goal_pose):
        goal = ObjectPursuitGoal()
        goal.pose = goal_pose
        self.pursuit_client.send_goal(goal)

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

    def cancel_pursuit_goal(self):
        self.pursuit_client.cancel_all_goals()
    
    def cancel(self):
        self.toggle_local_costmap(True)
        self.cancel_move_base()
        self.cancel_pursuit_goal()
        self.set_planner_to_default()
        self.set_linear_z(float("nan"), self.fast_stepper_speed)

    def run(self):
        rospy.spin()

if __name__ == "__main__":
    try:
        node = CentralPlanning()
        node.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        rospy.loginfo("Exiting central_planning node")
