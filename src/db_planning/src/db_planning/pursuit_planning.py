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

        self.timeout_safety_factor = rospy.get_param("~timeout_safety_factor", 3.0)
        self.angle_tolerance = rospy.get_param("~angle_tolerance", 0.05)
        self.position_tolerance = rospy.get_param("~position_tolerance", 0.05)
        self.min_linear_speed = rospy.get_param("~min_linear_speed", 0.07)
        self.max_linear_speed = rospy.get_param("~max_linear_speed", 0.2)
        self.min_angular_speed = rospy.get_param("~min_angular_speed", 1.0)
        self.max_angular_speed = rospy.get_param("~max_angular_speed", 3.0)
        self.zero_epsilon = rospy.get_param("~zero_epsilon", 1E-3)
        self.stabilization_timeout = rospy.Duration(rospy.get_param("~stabilization_timeout_s", 1.0))

        self.steer_kP = rospy.get_param("~steer_kP", 3.0)
        self.fine_steer_kP = rospy.get_param("~fine_steer_kP", 2.0)
        self.linear_kP = rospy.get_param("~linear_kP", 10.0)

        self.goal_pose = Pose2d()

        self.twist_lookup_avg_interval = 1.0 / 30.0
        self.timeout_turn_fudge = 0.025
        self.timeout_fudge = 1.0

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=100)

        self.pursuit_goal_sub = rospy.Subscriber("pursuit_goal", PoseStamped, self.goal_update_callback, queue_size=10)

        self.pursuit_action_server = actionlib.SimpleActionServer(self.pursuit_action_name, ObjectPursuitAction, self.pursuit_action_callback, auto_start=False)
        self.pursuit_action_server.start()
        rospy.loginfo("[%s] server started" % self.pursuit_action_name)

    def pursuit_action_callback(self, goal):
        self.set_goal(goal.pose)
        result = ObjectPursuitResult()
        result_state = self.run_pursuit()

        result.status = result_state
        if result_state == "success":
            self.pursuit_action_server.set_succeeded(result)
        elif result_state == "preempted":
            self.pursuit_action_server.set_preempted(result)
        else:  # failure
            self.pursuit_action_server.set_aborted(result)

    def run_pursuit(self):
        self.stop_motors()
        rospy.sleep(0.25)  # wait for motors to settle if coming from another navigation scheme
        # assumes self.goal_pose is set
        result_state = self.turn_towards_heading() 
        if result_state == "success":
            result_state = self.pursue_object()
        # if result_state == "success":
        #     result_state = self.turn_towards_object()
        return result_state

    def turn_towards_heading(self):
        state = self.get_state()
        target_angle = self.goal_pose.heading(state)
        error = Pose2d.normalize_theta(target_angle - state.theta)
        timeout = abs(error) / self.min_angular_speed * self.timeout_safety_factor + self.stabilization_timeout.to_sec() + self.timeout_turn_fudge
        rospy.loginfo("Turning %0.2f rad towards goal. Pursuing for %0.2fs at most" % (error, timeout))
        start_time = rospy.Time.now()
        timeout_duration = rospy.Duration(timeout)

        clock_rate = rospy.Rate(30.0)
        while True:
            clock_rate.sleep()
            if rospy.is_shutdown() or self.pursuit_action_server.is_preempt_requested():
                rospy.loginfo("[%s] preempted" % self.pursuit_action_name)
                self.stop_motors()
                return "preempted"
            
            state = self.get_state()
            target_angle = self.goal_pose.heading(state)
            error = Pose2d.normalize_theta(target_angle - state.theta)

            if abs(error) < self.angle_tolerance:
                self.stop_motors()
                print("Turn towards error: %s" % (self.goal_pose - state))
                return "success"
            
            if rospy.Time.now() - start_time > timeout_duration:
                rospy.logwarn("Timeout reaching. Giving up on turning towards object")
                self.stop_motors()
                return "failure"
            
            ang_v = self.steer_kP * error
            self.set_twist(0.0, ang_v)

    def pursue_object(self):
        state = self.get_state()
        error = self.goal_pose.relative_to(state)
        target_angle = self.goal_pose.heading(state)
        error.theta = Pose2d.normalize_theta(target_angle - state.theta)

        timeout = abs(error.x) / self.min_linear_speed * self.timeout_safety_factor + self.stabilization_timeout.to_sec() + self.timeout_fudge
        rospy.loginfo("Driving towards goal: %s. Pursuing for %0.2fs at most" % (error, timeout))
        start_time = rospy.Time.now()
        timeout_duration = rospy.Duration(timeout)
        success_time = None

        clock_rate = rospy.Rate(30.0)
        while True:
            clock_rate.sleep()
            now = rospy.Time.now()
            if rospy.is_shutdown() or self.pursuit_action_server.is_preempt_requested():
                rospy.loginfo("[%s] preempted" % self.pursuit_action_name)
                self.stop_motors()
                return "preempted"
            
            state = self.get_state()
            error = self.goal_pose.relative_to(state)
            # target_angle = self.goal_pose.heading(state)
            error.theta = Pose2d.normalize_theta(target_angle - state.theta)
            # print(error)

            if abs(error.x) < self.position_tolerance:
                self.stop_motors()
                if success_time is None:
                    rospy.loginfo("Tolerance reached. Waiting for stablization")
                    success_time = now
            else:
                success_time = None
                
            if now - start_time > timeout_duration:
                rospy.logwarn("Timeout reaching. Giving up on pursuing object")
                self.stop_motors()
                return "failure"
            
            if success_time is not None:
                if now - success_time > self.stabilization_timeout:
                    print("Pursuit error: %s" % error)
                    return "success"
                continue
            
            err_t_ang_v = self.fine_steer_kP * error.theta
            err_y_ang_v = self.steer_kP * error.y
            ang_v = err_t_ang_v + err_y_ang_v
            linear_v = self.linear_kP * error.x

            # print("%0.2f\t%0.2f\t|\t%0.2f\t%0.2f" % (linear_v, ang_v, err_t_ang_v, err_y_ang_v))
            self.set_twist(linear_v, ang_v)
    
    def turn_towards_object(self):
        state = self.get_state()
        error = self.goal_pose.relative_to(state).theta
        timeout = abs(error) / self.min_angular_speed * self.timeout_safety_factor + self.stabilization_timeout.to_sec() + self.timeout_turn_fudge
        rospy.loginfo("Turning %0.2f rad towards final heading. Pursuing for %0.2fs at most" % (error, timeout))
        start_time = rospy.Time.now()
        timeout_duration = rospy.Duration(timeout)
        success_time = None

        clock_rate = rospy.Rate(30.0)
        while True:
            clock_rate.sleep()
            now = rospy.Time.now()
            if rospy.is_shutdown() or self.pursuit_action_server.is_preempt_requested():
                rospy.loginfo("[%s] preempted" % self.pursuit_action_name)
                self.stop_motors()
                return "preempted"
            
            state = self.get_state()
            error = self.goal_pose.relative_to(state).theta
            # print(error, self.goal_pose.theta, state.theta)

            if abs(error) < self.angle_tolerance:
                self.stop_motors()
                if success_time is None:
                    rospy.loginfo("Tolerance reached. Waiting for stablization")
                    success_time = now
            else:
                success_time = None
            
            if now - start_time > timeout_duration:
                rospy.logwarn("Timeout reaching. Giving up on pursuing object")
                self.stop_motors()
                return "failure"
            
            if success_time is not None:
                if now - success_time > self.stabilization_timeout:
                    print("Turn towards error: %s" % (self.goal_pose - state))
                    return "success"
                continue

            ang_v = self.steer_kP * error
            self.set_twist(0.0, ang_v)
    
    def stop_motors(self):
        self.set_twist(0.0, 0.0)

    def goal_update_callback(self, msg):
        self.set_goal(msg)

    def set_twist(self, linear_x, ang_v):
        if abs(ang_v) < self.min_angular_speed and self.zero_epsilon < abs(linear_x) < self.min_linear_speed:
            linear_x = math.copysign(self.min_linear_speed, linear_x)
        elif abs(linear_x) < self.zero_epsilon:
            linear_x = 0.0

        if abs(linear_x) < self.min_linear_speed and self.zero_epsilon < abs(ang_v) < self.min_angular_speed:
            ang_v = math.copysign(self.min_angular_speed, ang_v)
        elif abs(ang_v) < self.zero_epsilon:
            ang_v = 0.0
        
        if abs(linear_x) > self.max_linear_speed:
            linear_x = math.copysign(self.max_linear_speed, linear_x)
        if abs(ang_v) > self.max_angular_speed:
            ang_v = math.copysign(self.max_angular_speed, ang_v)

        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = ang_v
        # print("vx: %0.2f, vt: %0.2f" % (linear_x, ang_v))
        self.cmd_vel_pub.publish(msg)
    
    def set_goal(self, pose_stamped: PoseStamped):
        if pose_stamped.header.frame_id != self.map_frame:
            rospy.logerr("Goal does not match planner's frame. %s != %s" % (pose_stamped.header.frame_id, self.map_frame))
        self.goal_pose.x = pose_stamped.pose.position.x
        self.goal_pose.y = pose_stamped.pose.position.y
        self.goal_pose.theta = self.get_theta(pose_stamped.pose.orientation)

    def get_state(self) -> Pose2d:
        try:
            robot_map_tf = self.tf_buffer.lookup_transform(self.map_frame, self.base_link_frame, rospy.Time(0), rospy.Duration(1.0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn("Failed to look up %s to %s. %s" % (self.map_frame, self.base_link_frame, e))
            return None
        state = Pose2d()
        state.x = robot_map_tf.transform.translation.x
        state.y = robot_map_tf.transform.translation.y
        state.theta = self.get_theta(robot_map_tf.transform.rotation)
        return state

    def get_theta(self, quaternion):
        return tf_conversions.transformations.euler_from_quaternion((
            quaternion.x,
            quaternion.y,
            quaternion.z,
            quaternion.w
        ))[2]

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
        rospy.spin()

    def test(self):
        def goto_point(x, y, theta):
            pose = PoseStamped()
            pose.header.frame_id = self.map_frame
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation = self.get_quaternion(theta)

            self.set_goal(pose)
            result_state = self.run_pursuit()
            rospy.loginfo("Pursuit result: %s" % result_state)
        goto_point(0.25, 0.05, 0.0)
        # goto_point(0.0, 0.0, 0.0)

        # goto_point(0.5, 0.5, 0.0)
        # goto_point(0.5, -0.5, 0.0)
        # goto_point(0.0, 0.0, 0.0)


if __name__ == "__main__":
    try:
        node = PursuitPlanning()
        node.run()
        # node.test()
    except rospy.ROSInterruptException:
        pass
    finally:
        rospy.loginfo("Exiting %s node" % node.name)
