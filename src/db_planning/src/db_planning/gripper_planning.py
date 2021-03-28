#!/usr/bin/env python3

import math

import rospy
import actionlib

from db_planning.msg import GripperAction, GripperGoal, GripperResult
from db_planning.srv import GrabbingSrv, GrabbingSrvResponse
from db_parsing.msg import DodobotGripper, DodobotParallelGripper, DodobotFSRs

class GripperPlanner:
    """
    Class definition for gripper_planning ROS node.
    """

    def __init__(self):
        """Initializes ROS and necessary services and publishers.
        """

        rospy.init_node(
            "gripper_planning"
            # disable_signals=True
            # log_level=rospy.DEBUG
        )

        self.gripper_action_name = rospy.get_param("~gripper_action_name", "gripper_actions")
        self.default_force_threshold = rospy.get_param("~default_force_threshold", 30)
        self.gripper_closed_cmd = rospy.get_param("~gripper_closed_cmd", 180)
        self.gripper_max_dist = rospy.get_param("~gripper_max_dist", 0.08)

        self.result = GripperResult()

        self.parallel_gripper_pub = rospy.Publisher("parallel_gripper_cmd", DodobotParallelGripper, queue_size=100)

        self.parallel_gripper_sub = rospy.Subscriber("parallel_gripper", DodobotParallelGripper, self.parallel_gripper_callback, queue_size=100)
        self.fsr_sub = rospy.Subscriber("fsrs", DodobotFSRs, self.fsr_callback, queue_size=100)

        self.fsr_left = 0
        self.fsr_right = 0

        self.gripper_dist = self.gripper_max_dist
        self.parallel_gripper_msg = DodobotParallelGripper()

        self.gripper_grabbing_service_name = "gripper_grabbing_service"
        rospy.loginfo("Setting up service %s" % self.gripper_grabbing_service_name)
        self.gripper_grabbing_srv = rospy.Service(self.gripper_grabbing_service_name, GrabbingSrv, self.gripper_grabbing_callback)
        rospy.loginfo("[%s] %s service is ready" % (self.gripper_action_name, self.gripper_grabbing_service_name))

        self.gripper_server = actionlib.SimpleActionServer(self.gripper_action_name, GripperAction, self.gripper_action_callback, auto_start=False)
        self.gripper_server.start()
        rospy.loginfo("[%s] server started" % self.gripper_action_name)

        rospy.loginfo("[%s] --- Dodobot gripper planning is up! ---" % self.gripper_action_name)

    def fsr_callback(self, msg):
        self.fsr_left = msg.left
        self.fsr_right = msg.right

    def is_grabbing(self, force_threshold=None):
        if force_threshold is None:
            force_threshold = self.default_force_threshold
        return self.fsr_left > force_threshold or self.fsr_right > force_threshold

    def gripper_grabbing_callback(self, req):
        force_threshold = req.force_threshold
        if force_threshold < 0:
            force_threshold = self.default_force_threshold
        if self.is_grabbing(force_threshold):
            return GrabbingSrvResponse(True)
        else:
            return GrabbingSrvResponse(False)

    def gripper_action_callback(self, goal):
        distance = goal.grip_distance
        force_threshold = goal.force_threshold

        if math.isnan(distance):
            rospy.loginfo("No action required from gripper. Skipping.")
            self.result.success = True
            self.gripper_server.set_succeeded(self.result)
            return
        
        if math.isnan(force_threshold):
            force_threshold = self.default_force_threshold

        success = self.send_grab_cmd(distance, force_threshold)
        rospy.loginfo("Gripper result: %s" % (success))

        self.result.success = success
        self.gripper_server.set_succeeded(self.result)

    def parallel_gripper_callback(self, msg):
        self.gripper_dist = msg.distance

    def send_grab_cmd(self, distance, force_threshold):
        self.parallel_gripper_msg.distance = distance
        self.parallel_gripper_msg.force_threshold = force_threshold

        rospy.loginfo("Publishing gripper command: %s\t%s" % (distance, force_threshold))

        self.parallel_gripper_pub.publish(self.parallel_gripper_msg)

        return self.wait_for_gripper(distance, force_threshold)

    def is_gripper_goal_reached(self, distance, force_threshold):
        return self.gripper_dist >= distance or self.is_grabbing(force_threshold)

    def wait_for_gripper(self, distance, force_threshold):
        rate = rospy.Rate(15.0)
        gripper_cmd_send_time = rospy.Time.now()
        while True:
            if rospy.is_shutdown() or self.gripper_server.is_preempt_requested():
                rospy.loginfo("%s: Preempted" % self.gripper_action_name)
                self.gripper_server.set_preempted()
                return False

            if self.is_gripper_goal_reached(distance, force_threshold):
                return True
            if rospy.Time.now() - gripper_cmd_send_time > rospy.Duration(10.0):
                rospy.logerr("Timed out while waiting for gripper to grab!")
                return False

            rate.sleep()


if __name__ == "__main__":
    try:
        node = GripperPlanner()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass

    finally:
        rospy.loginfo("Exiting gripper_planning node")
