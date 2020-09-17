#!/usr/bin/python

import tf
import rospy

import math

from db_planning.pid import PID
from db_planning.msg import ChassisAction, ChassisGoal, ChassisResult

class ChassisPlanning:
    """
    Class definition for planning_base ROS node.
    """

    def __init__(self):
        """
        Initializes ROS and necessary services and publishers.
        """

        rospy.init_node(
            "chassis_planning"
            # disable_signals=True
            # log_level=rospy.DEBUG
        )
        self.tf_listener = tf.TransformListener()

        self.chassis_action_name = rospy.get_param("~chassis_action_name", "chassis_actions")

        self.chassis_server = actionlib.SimpleActionServer(self.chassis_action_name, ChassisAction, self.chassis_callback)
        self.chassis_server.start()

        self.result = ChassisResult()

        self.angle_pid = PID()
        self.dist_pid = PID()
        self.has_active_goal = False

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_a = 0.0


    def chassis_callback(self, goal):
        goal_x = goal.goal_x
        goal_y = goal.goal_y
        goal_angle = goal.goal_angle
        base_speed = goal.base_speed
        base_ang_v = goal.base_ang_v
        pos_tolerance = goal.pos_tolerance
        drive_forwards = goal.drive_forwards

        params = {}

        if not math.isnan(goal_angle):
            params["goal_x"] = goal_x

        if not math.isnan(goal_angle):
            params["goal_y"] = goal_y

        if not math.isnan(goal_angle):
            params["goal_angle"] = goal_angle

        if len(params) == 0:
            rospy.loginfo("No action required from chassis. Skipping.")
            self.result.success = True
            self.chassis_server.set_succeeded()

        if not math.isnan(base_speed):
            params["base_speed"] = base_speed

        if not math.isnan(base_ang_v):
            params["base_ang_v"] = base_ang_v

        if not math.isnan(pos_tolerance):
            params["pos_tolerance"] = pos_tolerance

        params["drive_forwards"] = drive_forwards

        rospy.loginfo("Sending parameters to chassis guidance: %s" % str(params))
        self.goto_pose(params)

        self.tf_listener.waitForTransform("/base_link", "/odom", rospy.Time(), rospy.Duration(30.0))

        success = self.run_action()

        self.result.success = success
        self.chassis_server.set_succeeded()

    def check_preempt(self):
        if self.chassis_server.is_preempt_requested():
            rospy.loginfo("%s: Preempted" % self.chassis_action_name)
            self.chassis_server.set_preempted()
            return True
        else:
            return False

    def goto_pose(self, params):
        pass


    def run_action(self):
        pass


    def run(self):
        rate = rospy.Rate(30.0)
        while not rospy.is_shutdown():
            rate.sleep()
            if not self.has_active_goal:
                continue
            try:
                trans, rot = self.tf_listener.lookupTransform("/base_link", "/odom", rospy.Time(0))
                self.current_x = trans[0]
                self.current_y = trans[1]
                self.current_a = tf.transformations.euler_from_quaternion(rot[0], rot[1], rot[2], rot[3])[2]

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue



if __name__ == "__main__":
    try:
        node = ChassisPlanning()
        node.run()
        # rospy.spin()

    except rospy.ROSInterruptException:
        pass

    finally:
        rospy.loginfo("Exiting planning_base node")
