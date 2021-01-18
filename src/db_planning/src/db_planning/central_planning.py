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

import tf
import tf2_ros
import tf_conversions
import tf2_geometry_msgs

import geometry_msgs

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

from nav_msgs.srv import GetPlan

from std_srvs.srv import Trigger, TriggerResponse

import std_msgs.msg

from db_planning.msg import ChassisAction, ChassisGoal, ChassisResult
from db_planning.msg import FrontLoaderAction, FrontLoaderGoal, FrontLoaderResult
from db_planning.msg import GripperAction, GripperGoal, GripperResult

from db_planning.msg import SequenceRequestAction, SequenceRequestGoal, SequenceRequestResult

from db_planning.srv import GrabbingSrv, GrabbingSrvResponse

from db_planning.sequence import Sequence


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

        self.force_threshold = rospy.get_param("~force_threshold", 30)

        self.chassis_action_name = rospy.get_param("~chassis_action_name", "chassis_actions")
        self.front_loader_action_name = rospy.get_param("~front_loader_action_name", "front_loader_actions")
        self.gripper_action_name = rospy.get_param("~gripper_action_name", "gripper_actions")
        self.move_base_namespace = rospy.get_param("~move_base_namespace", "/move_base")

        self.main_workspace_tf = rospy.get_param("~map_tf", "map")
        self.base_link_tf = rospy.get_param("~base_link_tf", "base_link")
        self.linear_base_tf = rospy.get_param("~linear_base_link_tf", "linear_base_link")
        self.camera_base_link_tf = rospy.get_param("~camera_base_link_tf", "tilt_base_link")
        self.gripper_link_tf = rospy.get_param("~gripper_link_tf", "gripper_link")

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # front loader action client
        self.front_loader_action = self.make_action_client(self.front_loader_action_name, FrontLoaderAction)

        # chassis action client
        self.front_loader_action = self.make_action_client(self.chassis_action_name, ChassisAction)

        # gripper action client
        self.gripper_action = self.make_action_client(self.gripper_action_name, GripperAction)

        # front loader ready service
        self.front_loader_ready_srv = self.make_service_client("front_loader_ready_service", Trigger)

        # chassis ready service
        self.chassis_ready_srv = self.make_service_client("chassis_ready_service", Trigger)

        # gripper ready service
        self.gripper_grabbing_srv = self.make_service_client("gripper_grabbing_service", GrabbingSrv)

        # move_base action client
        self.move_action_client = self.make_action_client(self.move_base_namespace, MoveBaseAction)
        
        # move_base make_plan service
        self.make_plan_srv = self.make_service_client(self.move_base_namespace + "/make_plan", GetPlan)

        self.tilter_pub = rospy.Publisher("tilter_orientation", geometry_msgs.msg.Quaternion, queue_size=100)
        self.sound_pub = rospy.Publisher("sounds", std_msgs.msg.String, queue_size=10)

        # central_planning sequence action server
        self.sequence_server = actionlib.SimpleActionServer("sequence_request", SequenceRequestAction, self.sequence_callback, auto_start=False)
        self.sequence_server.start()
        rospy.loginfo("[%s] Dodobot sequence server started" % self.node_name)

        rospy.loginfo("[%s] --- Dodobot central planning is up! ---" % self.node_name)
    
    def make_action_client(self, name, action):
        action = actionlib.SimpleActionClient(name, action)
        rospy.loginfo("Waiting for action server %s" % name)
        action.wait_for_server()
        rospy.loginfo("[%s] %s action server connected" % (self.node_name, name))
        return action
    
    def make_service_client(self, name, srv_type):
        self.__dict__[name + "_service_name"] = name
        rospy.loginfo("Waiting for service %s" % name)
        srv_obj = rospy.ServiceProxy(name, srv_type)
        rospy.loginfo("%s service is ready" % name)
        return srv_obj

    def sequence_callback(self, goal):
        pass

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
