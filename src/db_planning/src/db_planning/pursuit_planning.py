#!/usr/bin/env python3
import math
import rospy
import tf2_ros
import actionlib
import tf_conversions

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist

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

        self.timeout_safety_factor = rospy.get_param("~timeout_safety_factor", "3.0")
        self.angle_tolerance = rospy.get_param("~angle_tolerance", "0.05")
        self.position_tolerance = rospy.get_param("~position_tolerance", "0.05")
        self.max_linear_speed = rospy.get_param("~max_linear_speed", "0.3")
        self.max_angular_speed = rospy.get_param("~max_angular_speed", "0.5")

        self.steer_kP = rospy.get_param("~steer_kP", "1.0")
        self.linear_kP = rospy.get_param("~linear_kP", "1.0")

        self.goal_pose = Pose2d()

        self.twist_lookup_avg_interval = 1.0 / 30.0

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
        # assumes self.goal_pose is set
        result_state = self.turn_towards_object() 
        if result_state == "success":
            result_state = self.pursue_object()
        return result_state

    def turn_towards_object(self):
        state = self.get_state()
        target_angle = self.goal_pose.heading(state)
        error = target_angle - state.theta
        timeout = error.theta / self.max_angular_speed * self.timeout_safety_factor
        rospy.loginfo("Turning towards goal. Pursuing for %0.2fs at most" % (timeout))
        start_time = rospy.Time.now()
        timeout_duration = rospy.Duration(timeout)

        while True:
            if rospy.is_shutdown() or self.front_loader_server.is_preempt_requested():
                rospy.loginfo("[%s] preempted" % self.pursuit_action_name)
                return "preempted"
            
            state = self.get_state()
            target_angle = self.goal_pose.heading(state)
            error = target_angle - state.theta

            if abs(error.theta) < self.angle_tolerance:
                return "success"
            
            if rospy.Time.now() - start_time > timeout_duration:
                rospy.logwarn("Timeout reaching. Giving up on turning towards object")
                return "failure"
            
            ang_v = self.steer_kP * error.theta
            if abs(ang_v) > self.max_angular_speed:
                ang_v = math.copysign(self.max_angular_speed, ang_v)
            self.set_twist(0.0, ang_v)

    def pursue_object(self):
        state = self.get_state()
        error = self.goal_pose.relative_to(state)
        timeout = error.x / self.max_linear_speed * self.timeout_safety_factor
        rospy.loginfo("Driving towards goal. Pursuing for %0.2fs at most" % (timeout))
        start_time = rospy.Time.now()
        timeout_duration = rospy.Duration(timeout)

        while True:
            if rospy.is_shutdown() or self.front_loader_server.is_preempt_requested():
                rospy.loginfo("[%s] preempted" % self.pursuit_action_name)
                return "preempted"
            
            state = self.get_state()
            error = self.goal_pose.relative_to(state)

            if abs(error.theta) < self.angle_tolerance and abs(error.x) < self.position_tolerance:
                return "success"
            
            if rospy.Time.now() - start_time > timeout_duration:
                rospy.logwarn("Timeout reaching. Giving up on pursuing object")
                return "failure"
            
            ang_v = self.steer_kP * error.theta
            if abs(ang_v) > self.max_angular_speed:
                ang_v = math.copysign(self.max_angular_speed, ang_v)
            linear_v = self.linear_kP * error.x
            if abs(linear_v) > self.max_linear_speed:
                linear_v = math.copysign(self.max_linear_speed, linear_v)
            self.set_twist(linear_v, ang_v)

    def goal_update_callback(self, msg):
        self.set_goal(msg)

    def set_twist(self, linear_x, ang_v):
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = ang_v
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

    def get_quaternion(self, yaw):
        return tf_conversions.transformations.quaternion_from_euler(0.0, 0.0, yaw)

    def run(self):
        rospy.spin()

    def test(self):
        pose = PoseStamped()
        pose.header.frame_id = self.map_frame
        pose.pose.position.x = 0.0
        pose.pose.position.y = 0.0
        pose.pose.orientation = self.get_quaternion(0.0)

        self.set_goal(pose)
        result_state = self.run_pursuit()
        rospy.loginfo("Pursuit result: %s" % result_state)


if __name__ == "__main__":
    try:
        node = PursuitPlanning()
        node.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        rospy.loginfo("Exiting %s node" % node.name)
