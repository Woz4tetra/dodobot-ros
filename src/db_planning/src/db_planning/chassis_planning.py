#!/usr/bin/python

import rospy
import tf
import math
import traceback
import actionlib
import geometry_msgs
import dynamic_reconfigure.client

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry

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

        self.min_ang_v = 0.75  # rad/s, cold start minimum: 0.75
        self.min_speed = 0.025  # m/s

        self.chassis_action_name = rospy.get_param("~chassis_action_name", "chassis_actions")
        self.chassis_server = actionlib.SimpleActionServer(self.chassis_action_name, ChassisAction, self.chassis_callback, auto_start=False)
        self.chassis_server.start()
        rospy.loginfo("[%s] server started" % self.chassis_action_name)

        # wait for action client
        self.move_action_client = actionlib.SimpleActionClient("/move_base", MoveBaseAction)
        self.move_action_client.wait_for_server()
        rospy.loginfo("[%s] move_base action server connected" % self.chassis_action_name)

        self.dwa_planner_dyn_client = dynamic_reconfigure.client.Client("/move_base/DWAPlannerROS", timeout=30, config_callback=self.callback)
        self.dwa_planner_updated = False
        rospy.loginfo("[%s] DWAPlannerROS dynamic reconfigure server connected" % self.chassis_action_name)

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

    def callback(self, config):
        self.dwa_planner_updated = True
        rospy.loginfo("Config set to %s" % str(config))

    def update_move_base_config(self, **params):
        self.dwa_planner_dyn_client.update_configuration(params)

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

        if goal_angle is None:
            goal_angle = 0.0

        sign = 1 if drive_forwards else -1

        max_vel_trans = self.base_speed
        min_vel_trans = self.min_speed
        max_vel_x = self.base_speed
        min_vel_x = self.min_speed
        max_vel_theta = self.base_ang_v
        min_vel_theta = self.min_ang_v
        yaw_goal_tolerance = self.angle_tolerance
        xy_goal_tolerance = self.pos_tolerance
        self.dwa_planner_updated = False

        self.update_move_base_config(
            max_vel_trans=max_vel_trans,
            min_vel_trans=min_vel_trans,
            max_vel_x=max_vel_x,
            min_vel_x=min_vel_x,
            max_vel_theta=max_vel_theta,
            min_vel_theta=min_vel_theta,
            yaw_goal_tolerance=yaw_goal_tolerance,
            xy_goal_tolerance=xy_goal_tolerance,
        )

        pose_base_link = geometry_msgs.msg.Pose()
        pose_base_link.position.x = goal_x
        pose_base_link.position.y = goal_y
        if goal_angle is not None:
            goal_quat = tf.transformations.quaternion_from_euler(0.0, 0.0, goal_angle)
            pose_base_link.orientation.x = goal_quat[0]
            pose_base_link.orientation.y = goal_quat[1]
            pose_base_link.orientation.z = goal_quat[2]
            pose_base_link.orientation.w = goal_quat[3]

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = pose_base_link

        while not self.dwa_planner_updated:
            rospy.sleep(0.1)

        rospy.loginfo("Send goal to move_base: %s" % goal)
        # Sends the goal to the action server.
        self.move_action_client.send_goal(goal)

        # Waits for the server to finish performing the action.
        self.move_action_client.wait_for_result()

        # Get the result of executing the action
        move_base_result = self.move_action_client.get_result()

        return bool(move_base_result)


    def shutdown_hook(self):
        if self.move_action_client.get_state() == actionlib.SimpleGoalState.ACTIVE:
            self.move_action_client.cancel_goal()
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
