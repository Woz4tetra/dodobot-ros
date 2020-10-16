#!/usr/bin/python
from __future__ import print_function

import os
import tf
import cv2
import math
import rospy
import datetime
import actionlib
import geometry_msgs
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

import db_planning.msg
from db_planning.msg import ChassisAction, ChassisGoal
from db_planning.msg import FrontLoaderAction, FrontLoaderGoal
from db_planning.sequence import Sequence
from db_planning.sounds import Sounds



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
        rospy.on_shutdown(self.shutdown_hook)

        self.bridge = CvBridge()

        self.audio_sink = rospy.get_param("~audio_sink", "alsa_output.usb-Generic_USB2.0_Device_20130100ph0-00.analog-stereo")

        self.sounds = Sounds(self.audio_sink, {
            "sequence_finished": "/home/ben/Music/sound_effects/ding.wav",
            "action_finished": "/home/ben/Music/sound_effects/click.wav",
            "action_failed": "/home/ben/Music/sound_effects/thud.wav",
        })

        self.insert_sequence_path = rospy.get_param("~insert_sequence_path", "./insert.csv")
        self.extract_sequence_path = rospy.get_param("~extract_sequence_path", "./extract.csv")
        self.insert_sequence = Sequence.from_path(self.insert_sequence_path)
        self.extract_sequence = Sequence.from_path(self.extract_sequence_path)
        self.sequence_types = ["insert", "extract"]

        self.chassis_action_name = rospy.get_param("~chassis_action_name", "chassis_actions")
        self.front_loader_action_name = rospy.get_param("~front_loader_action_name", "front_loader_actions")

        # create tag watching tf_listener
        self.tag_name = rospy.get_param("~tag_name", "target")  # tag's TF name
        self.base_start_name = rospy.get_param("~base_start_name", "base_start_link")
        self.tag_tf_name = self.tag_name  # "/" + self.tag_name
        self.base_start_tf_name = self.base_start_name  # "/" + self.base_start_name
        self.tf_listener = tf.TransformListener()

        self.saved_start_pos = None
        self.saved_start_quat = None
        self.last_saved_time = rospy.Time.now()

        self.debug_sequence_planning = False
        # self.debug_sequence_planning = True

        self.debug_move_base = False
        # self.debug_move_base = True

        self.tag_detections_topic = "/tag_detections_image"
        self.debug_image_dir = rospy.get_param("~debug_image_dir", ".")
        self.debug_image_date_format = "%Y-%m-%dT%H-%M-%S--%f"

        # create base_move action server
        self.sequence_server = actionlib.SimpleActionServer("/sequence_request", db_planning.msg.SequenceRequestAction, self.sequence_callback, auto_start=False)
        self.sequence_server.start()
        rospy.loginfo("[%s] Dodobot sequence server started" % self.node_name)

        # chassis and front loader services
        self.chassis_action = actionlib.SimpleActionClient(self.chassis_action_name, ChassisAction)
        self.chassis_action.wait_for_server()
        rospy.loginfo("[%s] %s action server connected" % (self.node_name, self.chassis_action_name))

        self.front_loader_action = actionlib.SimpleActionClient(self.front_loader_action_name, FrontLoaderAction)
        self.front_loader_action.wait_for_server()
        rospy.loginfo("[%s] %s action server connected" % (self.node_name, self.front_loader_action_name))

        # wait for action client
        self.move_action_client = actionlib.SimpleActionClient("/move_base", MoveBaseAction)
        self.move_action_client.wait_for_server()
        rospy.loginfo("[%s] move_base action server connected" % self.node_name)

        rospy.loginfo("[%s] --- Dodobot central planning is up! ---" % self.node_name)

        self.sounds["sequence_finished"].play()

    ### CALLBACK FUNCTIONS ###

    def sequence_callback(self, goal):
        # transform pose from robot frame to front loader frame
        sequence_type = goal.sequence_type
        rospy.loginfo("Sequence requested: %s" % sequence_type)
        result = db_planning.msg.SequenceRequestResult()

        if sequence_type not in self.sequence_types:
            rospy.logwarn("%s not an action type: %s" % (sequence_type, self.sequence_types))
            result.success = False
            self.sequence_server.set_aborted(result)
            return

        if not self.debug_sequence_planning:
            if not self.move_to_start_pose():
                result.success = False
                self.sequence_server.set_aborted(result)
                return

        # Temporary stopping point for function
        if self.debug_move_base:
            result.success = True
            self.sequence_server.set_succeeded(result)
            return

        rospy.sleep(2.0)
        self.search_for_tag(save_image=True)

        # TODO: pause rtabmap if running
        try:
            if sequence_type == "insert":
                seq_result = self.insert_action_sequence()
            elif sequence_type == "extract":
                seq_result = self.extract_action_sequence()
            else:
                raise RuntimeError("Invalid sequence type: %s" % sequence_type)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logwarn("Failed to complete action sequence due to TF error. Is the tag in view?: %s" % str(e))
            result.success = False
            self.sequence_server.set_aborted(result)
            return
        except BaseException as e:
            rospy.logwarn("Failed to complete action sequence: %s" % str(e))
            result.success = False
            self.sequence_server.set_aborted(result)
            return
        # TODO: resume rtabmap

        if seq_result is None:
            result.success = True
            self.sequence_server.set_succeeded(result)
        else:
            rospy.logwarn("Failed to complete action sequence. Stopped at index #%s" % seq_result)
            result.success = False
            self.sequence_server.set_aborted(result)

        rospy.loginfo("%s sequence completed!" % sequence_type)
        self.sounds["sequence_finished"].play()

    def move_to_start_pose(self):
        if self.saved_start_pos is None or self.saved_start_quat is None:
            rospy.loginfo("Saved position is null. Waiting 30s for a tag TF to appear.")
            self.tf_listener.waitForTransform("map", self.base_start_tf_name, rospy.Time(), rospy.Duration(30.0))
            self.search_for_tag()
            if self.saved_start_pos is None or self.saved_start_quat is None:
                rospy.logwarn("No tag visible or in memory. Can't direct the robot")
                return False

        # Creates a goal to send to the action server.
        pose_base_link = self.get_sequence_start_pose()

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = pose_base_link

        # Sends the goal to the action server.
        self.move_action_client.send_goal(goal)

        # Waits for the server to finish performing the action.
        self.move_action_client.wait_for_result()

        # Get the result of executing the action
        move_base_result = self.move_action_client.get_result()

        rospy.loginfo("move_base_result: %s, %s" % (type(move_base_result), move_base_result))
        if not move_base_result:
            rospy.logwarn("move_base failed to direct the robot to the directed position")
            result.success = False
            self.sequence_server.set_aborted(result)
            return False

        return True

    def insert_action_sequence(self):
        self.insert_sequence.reload()
        if self.debug_sequence_planning:
            adj_sequence = self.insert_sequence.sequence
        else:
            adj_sequence = self.adjust_sequence_into_odom(self.insert_sequence)
        for index, action in enumerate(adj_sequence):
            rospy.loginfo("\n\n--- Running insert action #%s: %s ---" % (index, action["comment"]))
            result = self.run_action(action)
            rospy.loginfo("--- Action #%s finished ---\n\n" % index)
            if not result:
                return index
        return None

    def extract_action_sequence(self):
        self.extract_sequence.reload()
        if self.debug_sequence_planning:
            adj_sequence = self.extract_sequence.sequence
        else:
            adj_sequence = self.adjust_sequence_into_odom(self.extract_sequence)
        for index, action in enumerate(adj_sequence):
            rospy.loginfo("\n\n--- Running extract action #%s: %s ---" % (index, action["comment"]))
            result = self.run_action(action)
            rospy.loginfo("--- Action #%s finished ---\n\n" % index)
            if not result:
                return index
        return None

    def run_action(self, action):
        rospy.loginfo("Sending action %s" % str(action))
        chassis_goal = self.get_goal_msg(ChassisGoal, action)
        front_loader_goal = self.get_goal_msg(FrontLoaderGoal, action)

        rospy.sleep(0.5)

        self.chassis_action.send_goal(chassis_goal) #, feedback_callback=self.chassis_action_progress)
        self.front_loader_action.send_goal(front_loader_goal) #, feedback_callback=self.front_loader_action_progress)

        self.front_loader_action.wait_for_result()
        rospy.loginfo("Front loader result received")
        front_loader_result = self.front_loader_action.get_result()
        rospy.loginfo("Front loader result obj: %s" % front_loader_result)
        rospy.loginfo("Front loader result: %s" % front_loader_result.success)
        if not front_loader_result.success:
            self.chassis_action.cancel_goal()
            return False

        self.chassis_action.wait_for_result()
        rospy.loginfo("Chassis result received")
        chassis_result = self.chassis_action.get_result()
        rospy.loginfo("Chassis result obj: %s" % chassis_result)
        rospy.loginfo("Chassis result: %s" % chassis_result.success)


        if not chassis_result.success:
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

    def get_sequence_start_pose(self):
        tag_pose = geometry_msgs.msg.Pose()
        rospy.loginfo("Using starting pos: %s, %s" % (self.saved_start_pos, self.saved_start_quat))
        tag_pose.position.x = self.saved_start_pos[0]
        tag_pose.position.y = self.saved_start_pos[1]
        tag_pose.position.z = self.saved_start_pos[2]
        tag_pose.orientation.x = 0.0  # self.saved_start_quat[0]
        tag_pose.orientation.y = 0.0  # self.saved_start_quat[1]
        tag_pose.orientation.z = self.saved_start_quat[2]
        tag_pose.orientation.w = self.saved_start_quat[3]
        # tag_pose.orientation.x = 0.0
        # tag_pose.orientation.y = 0.0
        # tag_pose.orientation.z = 0.0
        # tag_pose.orientation.w = 1.0

        return tag_pose

        # tfd_pose = self.tf_listener.transformPose(frame_id, tag_pose)
        # return tfd_pose

    def adjust_sequence_into_odom(self, sequence):
        # adjust all actions while the tag is in view. If it's not, throw an error
        adj_sequence = []
        for action in sequence.sequence:
            adj_sequence.append(self.get_action_goal_in_odom(action))
        return adj_sequence

    def get_action_goal_in_odom(self, action):
        # all action coordinates are relative to the base_start_link frame
        # and need to be transformed into the odom frame so goals can be sent.

        # The robot's starting position isn't needed since we're computing from
        # the theoretically perfect action_start_link which the robot should
        # be very close to

        tf_required = False

        pose = geometry_msgs.msg.PoseStamped()
        pose.header.frame_id = self.base_start_tf_name
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
            action_quaternion = tf.transformations.quaternion_from_euler(0.0, 0.0, action["goal_angle"])
            pose.pose.orientation.x = action_quaternion[0]
            pose.pose.orientation.y = action_quaternion[1]
            pose.pose.orientation.z = action_quaternion[2]
            pose.pose.orientation.w = action_quaternion[3]
            tf_required = True

        if not tf_required:
            return action

        tfd_pose = self.tf_listener.transformPose("/odom", pose)

        # goal_z=0.0 is when the gripper is grabbing the target. Offset defined in db_planning.launch (tag_to_start_pos)
        # This TF will translate goal_z to a Z height relative to the zero position of the linear slide
        tfd_linear_pose = self.tf_listener.transformPose("/linear_base_link", pose)

        tfd_action = {}
        tfd_action.update(action)
        if not math.isnan(action["goal_x"]) and not math.isnan(action["goal_y"]):
            tfd_action["goal_x"] = tfd_pose.pose.position.x
            tfd_action["goal_y"] = tfd_pose.pose.position.y
        if not math.isnan(action["goal_z"]):
            tfd_action["goal_z"] = tfd_linear_pose.pose.position.z

        if not math.isnan(action["goal_angle"]):
            tfd_goal_angle = tf.transformations.euler_from_quaternion([
                tfd_pose.pose.orientation.x,
                tfd_pose.pose.orientation.y,
                tfd_pose.pose.orientation.z,
                tfd_pose.pose.orientation.w
            ])
            tfd_action["goal_angle"] = tfd_goal_angle[2]
            # if the original goal_angle is NaN, that value will be copied over

        rospy.loginfo("x: %0.4f, y: %0.4f, z: %0.4f, a: %0.4f -> x: %0.4f, y: %0.4f, z: %0.4f, a: %0.4f" % (
            action["goal_x"], action["goal_y"], action["goal_z"], action["goal_angle"],
            tfd_action["goal_x"], tfd_action["goal_y"], tfd_action["goal_z"], tfd_action["goal_angle"],
        ))
        return tfd_action

    def search_for_tag(self, save_image=False):
        try:
            image_msg = rospy.wait_for_message(self.tag_detections_topic, Image, timeout=10.0)

            tag_trans, tag_rot = self.tf_listener.lookupTransform("map", self.tag_tf_name, rospy.Time(0))
            start_trans, start_rot = self.tf_listener.lookupTransform("map", self.base_start_tf_name, rospy.Time(0))
            self.saved_start_pos = start_trans
            self.saved_start_quat = tag_rot
            self.last_saved_time = rospy.Time.now()
            rospy.loginfo("Tag is visible. Base starting pose: (%0.4f, %0.4f, %0.4f). Saving location" % tuple(self.saved_start_pos))

            if save_image:
                cv2_img = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")

                timestamp = rospy.Time.now().to_sec()
                date_str = datetime.datetime.fromtimestamp(timestamp).strftime(self.debug_image_date_format)
                filename = date_str + ".png"
                debug_image_path = os.path.join(self.debug_image_dir, filename)
                cv2.imwrite(debug_image_path, cv2_img)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return
        except rospy.ROSException as e:
            rospy.logerr("Exception occurred in search_for_tag: %s" % str(e))
            # raise

    def run(self):
        rospy.loginfo("Tag locator task started")
        rate = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            self.search_for_tag()
            rospy.sleep(5.0)

            # if rospy.Time.now() - self.last_saved_time > rospy.Duration(300.0):
            #     rospy.loginfo("5 minutes elapsed. Saved tag position erased.")
            #     self.saved_start_pos = None
            #     self.saved_start_quat = None
            rate.sleep()

    def shutdown_hook(self):
        if self.move_action_client.get_state() == actionlib.SimpleGoalState.ACTIVE:
            self.move_action_client.cancel_goal()

if __name__ == "__main__":
    try:
        node = CentralPlanning()
        node.run()

    except rospy.ROSInterruptException:
        pass

    finally:
        rospy.loginfo("Exiting central_planning node")