#!/usr/bin/python

import rospy
import tf
import math
import traceback
import actionlib
import geometry_msgs
import dynamic_reconfigure.client

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

from db_planning.pid import PID
from db_planning.goal_controller import GoalController
from db_planning.msg import ChassisAction, ChassisGoal, ChassisResult
from db_planning.chassis_state import ChassisState


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
        rospy.on_shutdown(self.shutdown_hook)

        self.twist_command = geometry_msgs.msg.Twist()
        self.cmd_vel_pub = rospy.Publisher("cmd_vel", geometry_msgs.msg.Twist, queue_size=100)

        self.tf_listener = tf.TransformListener()

        self.result = ChassisResult()

        self.has_active_goal = False
        self.state = ChassisState()

        self.base_max_speed = 0.36  # m/s
        self.base_max_ang_v = 0.7  # rad/s

        self.pos_tolerance = 0.005  # m
        self.angle_tolerance = 0.01  # rad
        self.base_speed = self.base_max_speed
        self.base_ang_v = self.base_max_ang_v

        self.min_ang_v = 0.6  # rad/s
        self.min_speed = 0.05  # m/s

        self.controller = GoalController()
        # self.controller.set_constants(3.5, 8.0, -1.5)
        self.controller.set_constants(3.5, 6.0, -1.0)
        self.controller.forward_movement_only = True

        self.chassis_action_name = rospy.get_param("~chassis_action_name", "chassis_actions")
        self.chassis_server = actionlib.SimpleActionServer(self.chassis_action_name, ChassisAction, self.chassis_callback, auto_start=False)
        self.chassis_server.start()
        rospy.loginfo("[%s] server started" % self.chassis_action_name)

        rospy.loginfo("[%s] --- Dodobot chassis planning is up! ---" % self.chassis_action_name)


    def chassis_callback(self, goal):
        goal_x = goal.goal_x
        goal_y = goal.goal_y
        goal_angle = goal.goal_angle
        base_speed = goal.base_speed
        base_ang_v = goal.base_ang_v
        pos_tolerance = goal.pos_tolerance
        angle_tolerance = goal.angle_tolerance
        drive_forwards = goal.drive_forwards

        params = {}

        if not math.isnan(goal_x):
            params["goal_x"] = goal_x

        if not math.isnan(goal_y):
            params["goal_y"] = goal_y

        if not math.isnan(goal_angle):
            params["goal_angle"] = goal_angle

        if len(params) == 0:
            rospy.loginfo("No action required from chassis X: %s, Y: %s, A: %s %s. Skipping." % (goal_x, goal_y, goal_angle, params))
            self.set_succeeded(True)
            return

        if not math.isnan(base_speed):
            params["base_speed"] = base_speed

        if not math.isnan(base_ang_v):
            params["base_ang_v"] = base_ang_v

        if not math.isnan(pos_tolerance):
            params["pos_tolerance"] = pos_tolerance

        if not math.isnan(angle_tolerance):
            params["angle_tolerance"] = angle_tolerance

        params["drive_forwards"] = drive_forwards

        self.tf_listener.waitForTransform("/base_link", "/odom", rospy.Time(), rospy.Duration(5.0))

        self.has_active_goal = True
        self.update_current_pose()
        rospy.loginfo("Sending parameters to chassis guidance: %s" % str(params))
        try:
            success = self.goto_pose(params)
            self.send_cmd(0.0, 0.0)
            # success = self.send_goal(params)
        except BaseException as e:
            tb = traceback.format_exc()
            rospy.logerr("An error occurred while going to pose: %s" % str(tb))
            self.set_succeeded(False)
            return

        self.has_active_goal = False
        if not success:
            rospy.logerr("Failed to go to pose")

        self.set_succeeded(success)

    def set_succeeded(self, success):
        self.result.success = success
        self.chassis_server.set_succeeded(self.result)
        self.send_cmd(0.0, 0.0)

    def check_cancelled(self):
        if rospy.is_shutdown():
            self.send_cmd(0.0, 0.0)
            return True

        if self.chassis_server.is_preempt_requested():
            rospy.loginfo("%s: Preempted" % self.chassis_action_name)
            self.chassis_server.set_preempted()
            self.send_cmd(0.0, 0.0)
            return True
        else:
            return False

    def goto_pose(self, params):
        default_val = None  # float("nan")
        goal_x = params.get("goal_x", default_val)
        goal_y = params.get("goal_y", default_val)
        goal_angle = params.get("goal_angle", default_val)

        goal_state = ChassisState(goal_x, goal_y, goal_angle)
        rospy.loginfo("Current pose: %s. Goal pose: %s" % (self.state, goal_state))

        self.base_speed = params.get("base_speed", self.base_speed)
        self.base_ang_v = params.get("base_ang_v", self.base_ang_v)
        self.pos_tolerance = params.get("pos_tolerance", self.pos_tolerance)
        self.angle_tolerance = params.get("angle_tolerance", self.angle_tolerance)
        drive_forwards = bool(params.get("drive_forwards"))

        self.controller.max_linear_speed = self.base_speed
        self.controller.min_linear_speed = self.min_speed
        self.controller.max_angular_speed = self.base_ang_v
        self.controller.min_angular_speed = self.min_ang_v
        self.controller.linear_tolerance = self.pos_tolerance
        self.controller.angular_tolerance = self.angle_tolerance
        self.controller.reverse_direction = not drive_forwards

        rate = rospy.Rate(30.0)
        prev_time = rospy.Time.now()

        while not self.controller.at_goal(self.state, goal_state):
            rate.sleep()

            current_time = rospy.Time.now()
            dt = (current_time - prev_time).to_sec()
            prev_time = current_time

            command_state = self.controller.get_velocity(self.state, goal_state, dt)
            self.send_cmd(command_state.vx, command_state.vt)

            if self.controller.is_stuck():
                rospy.logwarn("Robot got stuck during go to pose routine!")
                return False

            if self.check_cancelled():
                rospy.loginfo("Go to pose routine cancelled")
                return False

        rospy.loginfo("goto_pose command completed successfully!")
        return True

    def send_cmd(self, linear_vx, angular_wz):
        self.twist_command.linear.x = linear_vx
        self.twist_command.angular.z = angular_wz
        rospy.loginfo("cmd: %s, %s" % (linear_vx, angular_wz))
        self.cmd_vel_pub.publish(self.twist_command)

    def update_current_pose(self):
        try:
            trans, rot = self.tf_listener.lookupTransform("/odom", "/base_link", rospy.Time(0))
            self.state.x = trans[0]
            self.state.y = trans[1]
            self.state.t = tf.transformations.euler_from_quaternion(rot)[2]

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return

    def run(self):
        rate = rospy.Rate(60.0)
        while not rospy.is_shutdown():
            rate.sleep()
            if not self.has_active_goal:
                rospy.sleep(0.1)
                continue
            self.update_current_pose()

    def shutdown_hook(self):
        if self.chassis_server.get_state() == actionlib.SimpleGoalState.ACTIVE:
            self.chassis_server.cancel_goal()

if __name__ == "__main__":
    try:
        node = ChassisPlanning()
        node.run()
        # rospy.spin()

    except rospy.ROSInterruptException:
        pass

    finally:
        rospy.loginfo("Exiting planning_base node")
