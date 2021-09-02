#!/usr/bin/env python3
import math
import rospy
import tf2_ros
import actionlib
import tf_conversions

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Quaternion

from nav_msgs.msg import Odometry

from db_planning.msg import ObjectPursuitAction, ObjectPursuitGoal, ObjectPursuitResult

from db_planning.robot_state import Pose2d
from db_planning.pursuit import PursuitManager
from db_planning.recursive_namespace import RecursiveNamespace

from db_planning.helpers import get_msg_properties


class PursuitPlanning:
    def __init__(self):
        self.name = "pursuit_planning"
        rospy.init_node(
            self.name,
            # log_level=rospy.DEBUG
        )
        self.pursuit_action_name = "pursuit_actions"
        self.map_frame = rospy.get_param("~global_frame", "map")
        self.base_link_frame = rospy.get_param("~robot_frame", "base_link")
        self.test_mode = rospy.get_param("~test_mode", False)

        self.pursuit_parameters = self.get_parameters()
        self.pursuit_manager = PursuitManager(
            self.pursuit_parameters,
            self.set_twist, self.get_state, self.should_stop
        )

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=100)

        self.pursuit_goal_sub = rospy.Subscriber("pursuit_goal", PoseStamped, self.goal_update_callback, queue_size=10)

        self.pursuit_action_server = actionlib.SimpleActionServer(self.pursuit_action_name, ObjectPursuitAction, self.pursuit_action_callback, auto_start=False)
        self.pursuit_action_server.start()
        rospy.loginfo("[%s] server started" % self.pursuit_action_name)
        if self.test_mode:
            rospy.loginfo("Running pursuit in test mode")

    def get_parameters(self, parameters: dict=None):
        if parameters is None:
            parameters = {}
        pursuit_parameters = RecursiveNamespace(
            timeout_safety_factor=2.0,  # How much to multiply the allotted time estimate
            angle_tolerance=0.05,  # +/- range of acceptable stopping angles
            position_tolerance=0.05,  # +/- range of acceptable stopping distances
            min_linear_speed=0.07,  # minimum linear command that produces chassis motion
            max_linear_speed=0.2,  # maximum allowed linear command
            min_angular_speed=1.0,  # minimum angular command that produces chassis motion
            max_angular_speed=3.0,  # maximum allowed angular command
            zero_epsilon=1E-3,  # command values to consider zero (prevents motor whining)
            stabilization_timeout=1.0,  # how long to wait for robot to settle on goal once it's reached
            steer_kP=6.0,  # proportion constant for turn commands based on error
            fine_steer_kP=2.0,  # proportion constant for fine turn commands based on error
            linear_kP=10.0,  # proportion constant for linear commands based on error
            timeout_turn_fudge=0.025,  # amount of extra time to give turn commands
            timeout_fudge=1.0,  # amount of extra time to give linear commands
            # if the robot exceeds this value in Y during drive_towards, jump to turn_towards
            loopback_y_tolerance=0.07,
            # if the robot exceeds this value in theta during drive_towards, jump to turn_towards
            loopback_theta_tolerance=0.1,
            forwards_motion_only=False,
            turn_towards_final_heading=False,
            reversed=False,
        )
        for name, value in parameters.items():
            if type(value) == float and math.isnan(value):
                continue
            if value is None:
                continue
            
            pursuit_parameters[name] = value

        for name, value in pursuit_parameters:
            pursuit_parameters[name] = rospy.get_param("~" + name, value)
        return pursuit_parameters

    def pursuit_action_callback(self, goal):
        parameters = self.get_parameters(get_msg_properties(goal))
        rospy.loginfo("Setting pursuit parameters: %s" % parameters)
        self.pursuit_manager.set_parameters(parameters)
        
        self.set_goal(goal.pose)
        result = ObjectPursuitResult()
        result_state = self.pursuit_manager.run()

        result.status = result_state
        if result_state == "success":
            rospy.loginfo("[%s] succeeded" % self.pursuit_action_name)
            self.pursuit_action_server.set_succeeded(result)
        elif result_state == "preempted":
            rospy.logwarn("[%s] preempted" % self.pursuit_action_name)
            self.pursuit_action_server.set_preempted(result)
        else:  # failure
            rospy.logerr("[%s] failed" % self.pursuit_action_name)
            self.pursuit_action_server.set_aborted(result)

    def should_stop(self):
        return rospy.is_shutdown() or self.pursuit_action_server.is_preempt_requested()

    def goal_update_callback(self, msg):
        self.set_goal(msg)

    def set_twist(self, linear_x, ang_v):
        if (abs(ang_v) < self.pursuit_parameters.min_angular_speed and
                self.pursuit_parameters.zero_epsilon < abs(linear_x) < self.pursuit_parameters.min_linear_speed):
            linear_x = math.copysign(self.pursuit_parameters.min_linear_speed, linear_x)
        elif abs(linear_x) < self.pursuit_parameters.zero_epsilon:
            linear_x = 0.0

        if (abs(linear_x) < self.pursuit_parameters.min_linear_speed and
                self.pursuit_parameters.zero_epsilon < abs(ang_v) < self.pursuit_parameters.min_angular_speed):
            ang_v = math.copysign(self.pursuit_parameters.min_angular_speed, ang_v)
        elif abs(ang_v) < self.pursuit_parameters.zero_epsilon:
            ang_v = 0.0
        
        if abs(linear_x) > self.pursuit_parameters.max_linear_speed:
            linear_x = math.copysign(self.pursuit_parameters.max_linear_speed, linear_x)
        if abs(ang_v) > self.pursuit_parameters.max_angular_speed:
            ang_v = math.copysign(self.pursuit_parameters.max_angular_speed, ang_v)

        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = ang_v
        self.cmd_vel_pub.publish(msg)
    
    def set_goal(self, pose_stamped: PoseStamped):
        if pose_stamped.header.frame_id != self.map_frame:
            rospy.logerr("Goal does not match planner's frame. %s != %s" % (pose_stamped.header.frame_id, self.map_frame))
        self.pursuit_manager.set_goal(
            pose_stamped.pose.position.x,
            pose_stamped.pose.position.y,
            Pose2d.theta_from_quat(pose_stamped.pose.orientation),
        )

    def get_state(self) -> Pose2d:
        try:
            robot_map_tf = self.tf_buffer.lookup_transform(self.map_frame, self.base_link_frame, rospy.Time(0), rospy.Duration(1.0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn("Failed to look up %s to %s. %s" % (self.map_frame, self.base_link_frame, e))
            return Pose2d.none_pose()
        state = Pose2d()
        state.x = robot_map_tf.transform.translation.x
        state.y = robot_map_tf.transform.translation.y
        state.theta = Pose2d.theta_from_quat(robot_map_tf.transform.rotation)
        return state

    def get_quaternion(self, yaw, as_list=False):
        quat = tf_conversions.transformations.quaternion_from_euler(0.0, 0.0, yaw)
        if as_list:
            return quat
        
        quat_msg = Quaternion()
        quat_msg.x = quat[0]
        quat_msg.y = quat[1]
        quat_msg.z = quat[2]
        quat_msg.w = quat[3]
        return quat_msg

    def run(self):
        if self.test_mode:
            self.test()
        else:
            rospy.spin()

    def test(self):
        def goto_point(x, y, theta, **kwargs):
            self.pursuit_manager.set_parameters(kwargs)
            rospy.loginfo("Parameters: %s" % self.pursuit_manager.pursuit_parameters)

            pose = PoseStamped()
            pose.header.frame_id = self.map_frame
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation = self.get_quaternion(theta)

            self.set_goal(pose)
            result_state = self.pursuit_manager.run()
            rospy.loginfo("Pursuit result: %s" % result_state)
        # goto_point(0.25, 0.05, 0.0)
        # goto_point(0.0, 0.0, 0.0)

        # goto_point(0.5, 0.5, 0.0)
        # goto_point(0.5, -0.5, 0.0)
        # goto_point(0.0, 0.0, 0.0)

        goto_point(0.5, 0.5, 0.0, reversed=True)
        goto_point(-0.5, 0.5, 0.0, reversed=True)
        goto_point(0.5, -0.5, 0.0, reversed=True)
        goto_point(-0.5, -0.5, 0.0, reversed=True)
        goto_point(0.0, 0.0, 0.0, reversed=True, turn_towards_final_heading=True)


if __name__ == "__main__":
    node = PursuitPlanning()
    try:
        node.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        rospy.loginfo("Exiting %s node" % node.name)
