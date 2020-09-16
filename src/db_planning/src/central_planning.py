#!/usr/bin/python

import tf
import rospy
import actionlib

from move_base.msg import MoveBaseAction, MoveBaseGoal

from db_planning.msg import SequenceRequestAction, SequenceRequestGoal
from db_planning.msg import ChassisAction, ChassisGoal
from db_planning.msg import FrontLoaderAction, FrontLoaderGoal
from db_planning.sequence import Sequence

class CentralPlanning:
    """
    Class definition for central_planning ROS node.
    """

    def __init__(self):
        """
        Initializes ROS and necessary services and publishers.
        """

        rospy.init_node(
            "central_planning"
            # disable_signals=True
            # log_level=rospy.DEBUG
        )

        self.insert_sequence_path = rospy.get_param("~insert_sequence_path", "./insert.csv")
        self.extract_sequence_path = rospy.get_param("~extract_sequence_path", "./extract.csv")
        self.insert_sequence = Sequence(self.insert_sequence_path)
        self.extract_sequence = Sequence(self.extract_sequence_path)
        self.action_types = ["insert", "extract"]

        self.chassis_action_name = rospy.get_param("~chassis_action_name", "chassis_actions")
        self.front_loader_action_name = rospy.get_param("~front_loader_action_name", "front_loader_actions")

        # create tag watching tf_listener
        self.tag_name = rospy.get_param("~tag_name", "tag")  # tag's TF name
        self.tag_name_stored = self.tag_name + "_stored"
        self.tf_listener = tf.Transformtf_listener()
        self.tf_broadcaster = tf.TransformBroadcaster()

        self.tag_saved_pos = None
        self.tag_saved_quat = None

        # create base_move action server
        self.sequence_server = actionlib.SimpleActionServer("sequence_request", SequenceRequestAction, self.sequence_callback)
        self.sequence_server.start()

        # chassis and front loader services
        self.chassis_action = actionlib.SimpleActionClient(self.chassis_action_name, ChassisAction)
        self.chassis_action.wait_for_server()

        self.front_loader_action = actionlib.SimpleActionClient(self.front_loader_action_name, FrontLoaderAction)
        self.front_loader_action.wait_for_server()

        # wait for action client
        self.move_action_client = actionlib.SimpleActionClient("move_base/goal", MoveBaseAction)
        self.move_action_client.wait_for_server()

    ### CALLBACK FUNCTIONS ###

    def sequence_callback(self, goal):
        # transform pose from robot frame to front loader frame
        action_type = goal.action_type
        result = SequenceRequestGoal()

        if action_type not in self.action_types:
            rospy.logwarn("%s not an action type: %s" % (action_type, self.action_types))
            result.success = False
            self.sequence_server.set_aborted(result)
            return

        # Creates a goal to send to the action server.
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.pose = pose_base_link

        # Sends the goal to the action server.
        self.move_action_client.send_goal(goal)

        # Waits for the server to finish performing the action.
        self.move_action_client.wait_for_result()

        # Get the result of executing the action
        result = self.move_action_client.get_result()

        print result
        if not result:
            rospy.logwarn("move_base failed to direct the robot to the directed position")
            result.success = False
            self.sequence_server.set_aborted(result)
            return

        # TODO: pause rtabmap if running

        if action_type == "insert":
            result = self.insert_action_sequence()
        elif action_type == "extract":
            result = self.extract_action_sequence()
        else:
            raise RuntimeError("Invalid action type: %s" % action_type)

        # TODO: resume rtabmap

        result.success = True
        self.sequence_server.set_succeeded(result)

    def insert_action_sequence(self):
        for action in self.insert_sequence:
            self.run_action(action)

    def extract_action_sequence(self):
        for action in self.extract_sequence:
            self.run_action(action)

    def run_action(self, action):
        chassis_goal = self.get_goal_msg(ChassisGoal, action)
        front_loader_goal = self.get_goal_msg(FrontLoaderGoal, action)

        self.chassis_action.send_goal_async(chassis_goal, feedback_callback=self.chassis_action_progress)
        self.front_loader_action.send_goal_async(front_loader_goal, feedback_callback=self.front_loader_action_progress)

        self.chassis_action.wait_for_result()
        self.front_loader_action.wait_for_result()

        chassis_result = self.chassis_action.get_result()
        front_loader_result = self.front_loader_action.get_result()

        if chassis_result.success and front_loader_result.success:
            return True
        else:
            return False

    def get_goal_msg(self, goal_msg_class, action):
        goal_msg = goal_msg_class()
        for key, value in action.items():
            setattr(goal_msg, key, value)
        return goal_msg

    def run(self):
        rate = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            try:
                trans, rot = tf_listener.lookupTransform("/map", "/" + self.tag_name, rospy.Time(0))
                self.tag_saved_pos = trans
                self.tag_saved_quat = rot
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue

            if self.tag_saved_pos is not None and self.tag_saved_quat is not None:
                now = rospy.Time.now()
                self.tf_broadcaster.sendTransform(
                    self.tag_saved_pos,
                    self.tag_saved_quat,
                    now,
                    "/map",
                    self.tag_name_stored,
                )
            rate.sleep()

if __name__ == "__main__":
    try:
        node = CentralPlanning()
        node.run()

    except rospy.ROSInterruptException:
        pass

    finally:
        rospy.loginfo("Exiting central_planning node")
