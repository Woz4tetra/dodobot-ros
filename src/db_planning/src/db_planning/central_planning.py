#!/usr/bin/python
from __future__ import print_function

import os
import cv2
import math
import datetime
import numpy as np

import rospy
import rosservice
import actionlib

import tf2_ros
import tf_conversions
import tf2_geometry_msgs

import geometry_msgs

from cv_bridge import CvBridge, CvBridgeError

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

from nav_msgs.msg import GetPlan

from std_srvs.srv import Trigger, TriggerResponse

from db_planning.msg import ChassisAction, ChassisGoal, ChassisResult
from db_planning.msg import FrontLoaderAction, FrontLoaderGoal, FrontLoaderResult
from db_planning.msg import GripperAction, GripperGoal, GripperResult

from db_planning.msg import SequenceRequestAction, SequenceRequestGoal, SequenceRequestResult

from db_planning.srv import GrabbingSrv, GrabbingSrvResponse

from db_planning.sequence import Sequence
from db_planning.sounds import Sounds


class SequenceState:
    def __init__(self):
        self.is_ready = False
        self.sequence_start_pose = None
        self.object_pose = None
        self.goal_tf_name = ""
        self.sequence_type = ""

    def reset(self):
        self.is_ready = False


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
        # rospy.on_shutdown(self.shutdown_hook)

        self.bridge = CvBridge()

        self.audio_sink = rospy.get_param("~audio_sink", "alsa_output.usb-Generic_USB2.0_Device_20130100ph0-00.analog-stereo")

        self.sounds = Sounds(self.audio_sink, {
            "sequence_finished": "/home/ben/Music/sound_effects/ding.wav",
            "action_finished": "/home/ben/Music/sound_effects/click.wav",
            "action_failed": "/home/ben/Music/sound_effects/thud.wav",
        })

        self.force_threshold = rospy.get_param("~force_threshold", 30)

        self.pickup_path = rospy.get_param("~pickup_sequence_path", "./pickup.yaml")
        self.delivery_path = rospy.get_param("~delivery_sequence_path", "./delivery.yaml")
        self.pickup_sequence = Sequence.from_path(self.pickup_path)
        self.delivery_sequence = Sequence.from_path(self.delivery_path)
        self.sequence_types = ["pickup", "delivery"]
        self.sequence_mapping = {
            "pickup": self.pickup_action_sequence,
            "delivery": self.delivery_action_sequence,
        }
        self.sequence_state = SequenceState()

        self.chassis_action_name = rospy.get_param("~chassis_action_name", "chassis_actions")
        self.front_loader_action_name = rospy.get_param("~front_loader_action_name", "front_loader_actions")
        self.gripper_action_name = rospy.get_param("~gripper_action_name", "gripper_actions")
        self.move_base_namespace = rospy.get_param("~move_base_namespace", "/move_base")

        self.main_workspace_tf = rospy.get_param("~map_tf", "map")
        self.base_link_tf = rospy.get_param("~base_link_tf", "base_link")
        self.linear_base_tf = rospy.get_param("~linear_base_link_tf", "linear_base_link")

        self.sequence_start_radius = rospy.get_param("~sequence_start_radius", 0.25)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # front loader action client
        self.front_loader_action = actionlib.SimpleActionClient(self.front_loader_action_name, FrontLoaderAction)
        rospy.loginfo("Waiting for action server %s" % self.front_loader_action_name)
        self.front_loader_action.wait_for_server()
        rospy.loginfo("[%s] %s action server connected" % (self.node_name, self.front_loader_action_name))

        # chassis action client
        self.chassis_action = actionlib.SimpleActionClient(self.chassis_action_name, ChassisAction)
        rospy.loginfo("Waiting for action server %s" % self.chassis_action_name)
        self.chassis_action.wait_for_server()
        rospy.loginfo("[%s] %s action server connected" % (self.node_name, self.chassis_action_name))

        # gripper action client
        self.gripper_action = actionlib.SimpleActionClient(self.gripper_action_name, GripperAction)
        rospy.loginfo("Waiting for action server %s" % self.gripper_action_name)
        self.gripper_action.wait_for_server()
        rospy.loginfo("[%s] %s action server connected" % (self.node_name, self.gripper_action_name))

        # front loader ready service
        self.front_loader_ready_service_name = "front_loader_ready_service"
        rospy.loginfo("Waiting for service %s" % self.front_loader_ready_service_name)
        self.front_loader_ready_srv = rospy.ServiceProxy(self.front_loader_ready_service_name, Trigger)
        rospy.loginfo("%s service is ready" % self.front_loader_ready_service_name)

        # chassis ready service
        self.chassis_ready_service_name = "chassis_ready_service"
        rospy.loginfo("Waiting for service %s" % self.chassis_ready_service_name)
        self.chassis_ready_srv = rospy.ServiceProxy(self.chassis_ready_service_name, Trigger)
        rospy.loginfo("%s service is ready" % self.chassis_ready_service_name)

        # gripper ready service
        self.gripper_grabbing_service_name = "gripper_grabbing_service"
        rospy.loginfo("Waiting for service %s" % self.gripper_grabbing_service_name)
        self.gripper_grabbing_srv = rospy.ServiceProxy(self.gripper_grabbing_service_name, GrabbingSrv)
        rospy.loginfo("%s service is ready" % self.gripper_grabbing_service_name)

        # move_base action client
        self.move_action_client = actionlib.SimpleActionClient(self.move_base_namespace, MoveBaseAction)
        rospy.loginfo("Waiting for service %s" % self.move_base_namespace)
        self.move_action_client.wait_for_server()
        rospy.loginfo("[%s] %s action server connected" % (self.node_name, self.move_base_namespace))

        # move_base make_plan service
        self.move_base_make_plan_service_name = self.move_base_namespace + "/make_plan"
        rospy.loginfo("Waiting for service %s" % self.move_base_make_plan_service_name)
        self.make_plan_srv = rospy.ServiceProxy(self.move_base_make_plan_service_name, GetPlan)
        rospy.loginfo("%s service is ready" % self.move_base_make_plan_service_name)

        self.tilter_pub = rospy.Publisher("tilter_orientation", geometry_msgs.msg.Quaternion, queue_size=100)

        # central_planning sequence action server
        self.sequence_server = actionlib.SimpleActionServer("sequence_request", SequenceRequestAction, self.sequence_callback, auto_start=False)
        self.sequence_server.start()
        rospy.loginfo("[%s] Dodobot sequence server started" % self.node_name)

        rospy.loginfo("[%s] --- Dodobot central planning is up! ---" % self.node_name)

    ### CALLBACK FUNCTIONS ###

    def sequence_callback(self, goal):
        sequence_type = goal.sequence_type  # type sequence to run (pickup or delivery)
        sequence_goal = goal.sequence_goal  # name of the TF to perform sequence on
        rospy.loginfo("Sequence requested: %s @ %s" % (sequence_type, sequence_goal))
        result = SequenceRequestResult()

        self.sequence_state.sequence_type = sequence_type
        self.sequence_state.goal_tf_name = sequence_goal

        if sequence_type not in self.sequence_types:
            rospy.logwarn("%s not an action type: %s" % (sequence_type, self.sequence_types))
            self.set_aborted()
            return

        if not self.check_sequence_prerequisites(sequence_type):
            self.set_aborted()
            return

        self.sequence_state.is_ready = True

        if not self.move_to_start_pose(sequence_goal):
            self.set_success(False)
            return

        self.look_at_object()
        rospy.Time.sleep(0.25)  # wait for tilter

        try:
            seq_result = self.sequence_mapping[sequence_type]()
        except BaseException as e:
            rospy.logwarn("Failed to complete action sequence: %s" % str(e))
            self.set_aborted()
            raise

        if seq_result is None:
            self.set_success(True)
            rospy.loginfo("%s sequence completed!" % sequence_type)
            self.sounds["sequence_finished"].play()
        else:
            self.set_success(False)
            rospy.logwarn("Failed to complete action sequence. Stopped at index #%s" % seq_result)

    def set_success(self, success):
        result = SequenceRequestResult()
        result.success = success
        self.sequence_server.set_succeeded(result)
        self.sequence_state.is_ready = False

    def set_aborted(self):
        result = SequenceRequestResult()
        result.success = False
        self.sequence_server.set_aborted(result)
        self.sequence_state.is_ready = False

    def check_sequence_prerequisites(self, sequence_type):
        # linear must be homed and motors active
        # for delivery, object must be in gripper
        # for pickup, no object must be in the gripper

        resp = self.front_loader_ready_srv()
        if not resp.success:
            rospy.logerr("Front loader not ready! %s" % str(resp.message))
            return False
        resp = self.chassis_ready_srv()
        if not resp.success:
            rospy.logerr("Chassis loader not ready! %s" % str(resp.message))
            return False

        resp = self.gripper_grabbing_srv(self.force_threshold)
        if sequence_type == "pickup" and resp.is_grabbing:
            rospy.logerr("Gripper is already grabbing something! Can't run pickup sequence")
            return False
        if sequence_type == "delivery" and not resp.is_grabbing:
            rospy.logerr("Gripper isn't grabbing anything! Can't run delivery sequence")
            return False

        return True

    def move_to_start_pose(self, tf_name):
        goal_tf = self.tf_buffer.lookup_transform(self.main_workspace_tf, tf_name, rospy.Time(0), rospy.Duration(1.0))
        start_tf = self.tf_buffer.lookup_transform(self.main_workspace_tf, self.base_link_tf, rospy.Time(0), rospy.Duration(1.0))

        start_pose = geometry_msgs.msg.Pose()
        start_pose.position = start_tf.transform.translation
        start_pose.orientation = start_tf.transform.rotation

        goal_pose = geometry_msgs.msg.Pose()
        goal_pose.position = goal_tf.transform.translation
        goal_pose.orientation = goal_tf.transform.rotation
        rospy.loginfo("Object pose: %s" % goal_pose)

        req_plan = GetPlan()
        req_plan.start = start_pose
        req_plan.goal = goal_pose
        req_plan.tolerance = self.sequence_start_radius
        nav_plan = make_plan_srv(req_plan)

        if len(nav_plan.poses) == 0:
            rospy.logerr("Failed to produce a path to the object!")
            return False

        goal_point = self.pose_to_array(goal_pose)
        for pose in nav_plan.poses[::-1]:  # iterate backwards through the plan
            path_point = self.pose_to_array(pose.pose)

            distance = np.linalg.norm(goal_point - path_point)
            if distance <= self.sequence_start_radius:
                move_base_goal_pose = pose

        rospy.loginfo("Sequence start pose: %s" % move_base_goal_pose)
        self.sequence_state.sequence_start_pose = move_base_goal_pose

        # set sequence start pose to the last pose's orientation in the path
        # since the object's orientation isn't known (yet!)
        last_pose = nav_plan.poses[-1]
        self.sequence_state.object_pose = goal_pose
        self.sequence_state.object_pose.orientation = last_pose.pose.orientation

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = self.main_workspace_tf
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = move_base_goal_pose

        self.move_action_client.send_goal(goal)
        self.move_action_client.wait_for_result()
        move_base_result = self.move_action_client.get_result()
        return bool(move_base_result)

    def look_at_object(self):
        camera_base_tf = self.tf_buffer.lookup_transform(
            self.main_workspace_tf,
            self.camera_base_link_tf,
            rospy.Time(0),
            rospy.Duration(1.0)
        )
        tfd_pose = tf2_geometry_msgs.do_transform_pose(self.sequence_state.object_pose, camera_base_tf)

        self.tilter_pub.publish(tfd_pose.pose.orientation)

    def pickup_action_sequence(self):
        self.pickup_sequence.reload()
        adj_sequence = self.adjust_sequence_into_main_tf(self.pickup_sequence)
        for index, action in enumerate(adj_sequence):
            rospy.loginfo("\n\n--- Running pickup action #%s: %s ---" % (index, action["comment"]))
            result = self.run_action(action)

            rospy.loginfo("--- Action #%s finished ---\n\n" % index)
            if not result:
                return index
        return None

    def delivery_action_sequence(self, sequence_goal):
        self.delivery_sequence.reload()
        adj_sequence = self.adjust_sequence_into_main_tf(self.delivery_sequence)
        for index, action in enumerate(adj_sequence):
            rospy.loginfo("\n\n--- Running delivery action #%s: %s ---" % (index, action["comment"]))
            result = self.run_action(action)
            rospy.loginfo("--- Action #%s finished ---\n\n" % index)
            if not result:
                return index
        return None

    def run_action(self, action):
        rospy.loginfo("Sending action %s" % str(action))
        chassis_goal = self.get_goal_msg(ChassisGoal, action)
        front_loader_goal = self.get_goal_msg(FrontLoaderGoal, action)
        gripper_goal = self.get_goal_msg(GripperGoal, action)

        rospy.sleep(0.5)

        self.chassis_action.send_goal(chassis_goal) #, feedback_callback=self.chassis_action_progress)
        self.front_loader_action.send_goal(front_loader_goal) #, feedback_callback=self.front_loader_action_progress)
        self.gripper_action.send_goal(gripper_goal)

        self.front_loader_action.wait_for_result()
        self.chassis_action.wait_for_result()
        self.gripper_action.wait_for_result()

        front_loader_result = self.front_loader_action.get_result()
        chassis_result = self.chassis_action.get_result()
        gripper_result = self.gripper_action.get_result()

        rospy.loginfo("Results: \n\tFront loader:\t%s \n\tChassis:\t%s \n\tGripper:\t%s" % (
            front_loader_result.success,
            chassis_result.success,
            gripper_result.success,
        ))

        if not front_loader_result.success or not chassis_result.success or not gripper_result.success:
            # self.front_loader_action.cancel_goal()
            # self.chassis_action.cancel_goal()
            # self.gripper_action.cancel_goal()
            self.sounds["action_failed"].play()
            return False

        self.sounds["action_finished"].play()
        return True

    def chassis_action_progress(self, msg):
        rospy.loginfo("x: %s, y: %s" % (msg.current_x, msg.current_y))

    def front_loader_action_progress(self, msg):
        rospy.loginfo("z: %s" % (msg.current_z))

    def get_goal_msg(self, goal_msg_class, action):
        goal_msg = goal_msg_class()
        for key, value in action.items():
            try:
                setattr(goal_msg, key, value)
            except AttributeError:
                pass
        return goal_msg

    def adjust_sequence_into_main_tf(self, sequence):
        # adjust all actions while the tag is in view. If it's not, throw an error
        adj_sequence = []

        sequence_goal = self.sequence_state.object_pose

        main_transform = self.tf_buffer.lookup_transform(self.main_workspace_tf,
            sequence_goal,  # source frame
            rospy.Time(0),  # get the tf at first available time
            rospy.Duration(1.0)  # wait for 1 second
        )
        linear_transform = self.tf_buffer.lookup_transform(self.linear_base_tf,
            sequence_goal,  # source frame
            rospy.Time(0),  # get the tf at first available time
            rospy.Duration(1.0)  # wait for 1 second
        )

        for action in sequence.sequence:
            adj_sequence.append(self.get_action_goal_in_main_tf(sequence_goal, main_transform, linear_transform, action))
        return adj_sequence

    def pose_to_array(self, pose):
        position = pose.position
        return np.array([position.x, position.y, position.z])

    def quat_to_z_angle(self, orientation):
        tfd_goal_angle = tf_conversions.transformations.euler_from_quaternion([
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w
        ])
        return tfd_goal_angle[2]

    def get_action_goal_in_main_tf(self, sequence_goal, main_transform, linear_transform, action):
        # all action coordinates are relative to the object's frame
        # (with orientation set relative to map since I haven't figured out how
        # to get object orientation yet)
        # and need to be transformed into the main tf frame so goals can be sent.

        # The robot's starting position isn't needed since we're computing from
        # the object's frame. The robot will try to follow the path as closely
        # as possible

        pose = geometry_msgs.msg.PoseStamped()
        pose.header.frame_id = sequence_goal
        if not math.isnan(action["goal_x"]) and not math.isnan(action["goal_y"]):
            pose.pose.position.x = action["goal_x"]
            pose.pose.position.y = action["goal_y"]
            tf_required = True

        if not math.isnan(action["goal_z"]):
            pose.pose.position.z = action["goal_z"]
            tf_required = True

        if math.isnan(action["goal_angle"]):
            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 0.0
            pose.pose.orientation.z = 0.0
            pose.pose.orientation.w = 1.0
        else:
            action_quaternion = tf_conversions.transformations.quaternion_from_euler(0.0, 0.0, action["goal_angle"])
            pose.pose.orientation.x = action_quaternion[0]
            pose.pose.orientation.y = action_quaternion[1]
            pose.pose.orientation.z = action_quaternion[2]
            pose.pose.orientation.w = action_quaternion[3]
            tf_required = True

        tfd_pose = tf2_geometry_msgs.do_transform_pose(pose, main_transform)

        # goal_z=0.0 is when the gripper is grabbing the target. Offset defined in db_planning.launch (tag_to_start_pos)
        # This TF will translate goal_z to a Z height relative to the zero position of the linear slide
        tfd_linear_pose = tf2_geometry_msgs.do_transform_pose(pose, linear_transform)

        tfd_action = {}
        tfd_action.update(action)
        if not math.isnan(action["goal_x"]) and not math.isnan(action["goal_y"]):
            tfd_action["goal_x"] = tfd_pose.pose.position.x
            tfd_action["goal_y"] = tfd_pose.pose.position.y
        if not math.isnan(action["goal_z"]):
            tfd_action["goal_z"] = tfd_linear_pose.pose.position.z

        if not math.isnan(action["goal_angle"]):
            tfd_action["goal_angle"] = self.quat_to_z_angle(tfd_pose.pose.orientation)
            # if the original goal_angle is NaN, that value will be copied over

        rospy.loginfo("x: %0.4f, y: %0.4f, z: %0.4f, a: %0.4f -> x: %0.4f, y: %0.4f, z: %0.4f, a: %0.4f" % (
            action["goal_x"], action["goal_y"], action["goal_z"], action["goal_angle"],
            tfd_action["goal_x"], tfd_action["goal_y"], tfd_action["goal_z"], tfd_action["goal_angle"],
        ))
        return tfd_action

    def look_at_object_demo(self):
        rate = rospy.Rate(10.0)
        tf_name = "Orange_0"
        while not rospy.is_shutdown():
            goal_tf = self.tf_buffer.lookup_transform(self.main_workspace_tf, tf_name, rospy.Time(0), rospy.Duration(1.0))
            goal_pose = geometry_msgs.msg.Pose()
            goal_pose.position = goal_tf.transform.translation
            goal_pose.orientation = goal_tf.transform.rotation
            self.sequence_state.object_pose = goal_pose

            self.look_at_object()

            rate.sleep()

    def run(self):
        self.look_at_object_demo()
        # rospy.spin()

    def shutdown_hook(self):
        pass

if __name__ == "__main__":
    try:
        node = CentralPlanning()
        node.run()

    except rospy.ROSInterruptException:
        pass

    finally:
        rospy.loginfo("Exiting central_planning node")
