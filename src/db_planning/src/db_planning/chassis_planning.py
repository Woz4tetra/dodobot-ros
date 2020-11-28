#!/usr/bin/python

import math
import pprint
import traceback

import rospy
import tf2_ros
import tf_conversions
import actionlib
import geometry_msgs
import dynamic_reconfigure.client

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry

from std_srvs.srv import Trigger, TriggerResponse

from db_planning.msg import ChassisAction, ChassisGoal, ChassisResult

from db_planning.pid import PID
from db_planning.goal_controller import GoalController
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

        self.result = ChassisResult()

        self.base_max_speed = 0.36  # m/s, absolute max: 0.36
        self.base_max_ang_v = 7.0  # rad/s, absolute max: 7.0

        self.pos_tolerance = 0.01  # m
        self.angle_tolerance = 0.05  # rad
        self.base_speed = self.base_max_speed
        self.base_ang_v = self.base_max_ang_v

        self.min_ang_v = 0.1  # rad/s, cold start minimum: 0.75
        self.min_speed = 0.01  # m/s

        self.base_link_frame = "base_link"
        # self.base_link_frame = "gripper_link"  # plan based on the gripper position
        self.map_frame = "map"

        self.chassis_action_name = rospy.get_param("~chassis_action_name", "chassis_actions")
        self.chassis_server = actionlib.SimpleActionServer(self.chassis_action_name, ChassisAction, self.chassis_callback, auto_start=False)
        self.chassis_server.start()
        rospy.loginfo("[%s] server started" % self.chassis_action_name)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.state = ChassisState()

        self.controller = GoalController()
        # self.controller.set_constants(3.5, 8.0, -1.5)
        self.controller.set_constants(3.5, 20.0, -1.0)
        self.controller.forward_movement_only = False

        self.chassis_ready_service_name = "chassis_ready_service"
        rospy.loginfo("Setting up service %s" % self.chassis_ready_service_name)
        self.chassis_ready_srv = rospy.Service(self.chassis_ready_service_name, Trigger, self.chassis_ready_callback)
        rospy.loginfo("%s service is ready" % self.chassis_ready_service_name)

        rospy.loginfo("[%s] --- Dodobot chassis planning is up! ---" % self.chassis_action_name)

    def chassis_ready_callback(self, req):
        # TODO: pull is_active flag from firmware
        return TriggerResponse(True, "Ready!")
        # if self.is_active:
        #     return TriggerResponse(True, "Ready!")
        # else:
        #     return TriggerResponse(False, "active: %s" % str(self.is_active))

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

        rospy.loginfo("Sending parameters to chassis guidance: %s" % str(params))
        try:
            success = self.goto_pose(params)
            self.send_stop()
        except BaseException as e:
            tb = traceback.format_exc()
            rospy.logerr("An error occurred while going to pose: %s" % str(tb))
            self.set_succeeded(False)
            return

        if not success:
            rospy.logerr("Failed to go to pose")

        self.set_succeeded(success)

    def send_stop(self):
        rospy.loginfo("Chassis planner is sending stop")
        self.send_cmd(0.0, 0.0)

    def send_cmd(self, linear_vx, angular_wz):
        self.twist_command.linear.x = linear_vx
        self.twist_command.angular.z = angular_wz
        rospy.loginfo("cmd: %s, %s" % (self.twist_command.linear.x, self.twist_command.angular.z))
        self.cmd_vel_pub.publish(self.twist_command)

    def set_succeeded(self, success):
        self.result.success = success
        self.chassis_server.set_succeeded(self.result)
        self.send_stop()

    def check_cancelled(self):
        if rospy.is_shutdown():
            self.send_stop()
            return True

        if self.chassis_server.is_preempt_requested():
            rospy.loginfo("%s: Preempted" % self.chassis_action_name)
            self.chassis_server.set_preempted()
            self.send_stop()
            return True
        else:
            return False

    def local_planner_callback(self, config):
        self.local_planner_updated = True
        rospy.loginfo("Config set to %s" % pprint.pformat(dict(config)))

    def update_move_base_config(self, **params):
        self.local_planner_dyn_client.update_configuration(params)

    def update_current_pose(self):
        try:
            goal_tf = self.tf_buffer.lookup_transform(self.map_frame, self.base_link_frame, rospy.Time(0))
            self.state.x = goal_tf.transform.translation.x
            self.state.y = goal_tf.transform.translation.y
            self.state.t = tf_conversions.transformations.euler_from_quaternion([
                goal_tf.transform.rotation.x,
                goal_tf.transform.rotation.y,
                goal_tf.transform.rotation.z,
                goal_tf.transform.rotation.w
            ])[2]

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            return

    def goto_pose(self, params):
        default_val = None  # float("nan")
        goal_x = params.get("goal_x", default_val)
        goal_y = params.get("goal_y", default_val)
        goal_angle = params.get("goal_angle", default_val)

        self.base_speed = params.get("base_speed", self.base_speed)
        self.base_ang_v = params.get("base_ang_v", self.base_ang_v)
        self.pos_tolerance = params.get("pos_tolerance", self.pos_tolerance)
        self.angle_tolerance = params.get("angle_tolerance", self.angle_tolerance)
        drive_forwards = bool(params.get("drive_forwards"))

        goal_state = ChassisState(goal_x, goal_y, goal_angle)
        rospy.loginfo("Current pose: %s. Goal pose: %s" % (self.state, goal_state))

        self.controller.max_linear_speed = self.base_speed
        self.controller.min_linear_speed = self.min_speed
        self.controller.max_angular_speed = self.base_ang_v
        self.controller.min_angular_speed = self.min_ang_v
        self.controller.linear_tolerance = self.pos_tolerance
        self.controller.angular_tolerance = self.angle_tolerance
        self.controller.reverse_direction = not drive_forwards

        rate = rospy.Rate(60.0)
        prev_time = rospy.Time.now()

        was_at_goal = False
        am_at_goal_timeout = rospy.Duration(0.25)
        am_at_goal_timer = rospy.Time.now()

        while True:
            rate.sleep()
            self.update_current_pose()

            is_at_goal = self.controller.at_goal(self.state, goal_state)
            if is_at_goal:
                break
            # if is_at_goal and abs(command_state.vx) < 0.001 and abs(command_state.vt) < 0.001:
            #     break
            # if is_at_goal:
            #     if is_at_goal != was_at_goal:
            #         am_at_goal_timer = rospy.Time.now()
            #         was_at_goal = is_at_goal
            #         self.send_stop()
            #     if rospy.Time.now() - am_at_goal_timer > am_at_goal_timeout:
            #         break
            # else:
            #     was_at_goal = is_at_goal

            current_time = rospy.Time.now()
            dt = (current_time - prev_time).to_sec()
            prev_time = current_time

            command_state = self.controller.get_velocity(self.state, goal_state, dt)
            self.send_cmd(command_state.vx, command_state.vt)

            if self.controller.is_stuck():
                rospy.logwarn("Robot got stuck during go to pose routine!")
                rospy.loginfo("goal: %s, current: %s, error: %s" % (goal_state, self.state, goal_state - self.state))
                return False

            if self.check_cancelled():
                rospy.loginfo("Go to pose routine cancelled")
                return False

        rospy.loginfo("goto_pose command completed successfully!")
        rospy.loginfo("goal: %s, current: %s, error: %s" % (goal_state, self.state, goal_state - self.state))
        return True


    def shutdown_hook(self):
        # if self.move_action_client.get_state() == actionlib.SimpleGoalState.ACTIVE:
        #     self.move_action_client.cancel_goal()
        if self.chassis_server.is_active():
            self.set_succeeded(False)

if __name__ == "__main__":
    try:
        node = ChassisPlanning()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass

    finally:
        rospy.loginfo("Exiting planning_base node")
