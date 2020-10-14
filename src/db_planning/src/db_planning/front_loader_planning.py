#!/usr/bin/python

import rospy
import tf
import math
import Queue
import actionlib

from geometry_msgs.msg import Pose, TransformStamped

from db_planning.msg import FrontLoaderAction, FrontLoaderGoal, FrontLoaderResult
from db_parsing.msg import DodobotLinear, DodobotLinearEvent, DodobotGripper, DodobotParallelGripper


class FrontLoaderPlanner:
    """Class definition for front_loader_planning ROS node.
    """

    LINEAR_EVENTS = {
        1: "ACTIVE_TRUE",
        2: "ACTIVE_FALSE",
        3: "HOMING_STARTED",
        4: "HOMING_FINISHED",
        5: "MOVE_STARTED",
        6: "MOVE_FINISHED",
        7: "POSITION_ERROR",
        8: "NOT_HOMED",
        9: "NOT_ACTIVE",
    }

    FAILED_GOAL_EVENTS = ("ACTIVE_FALSE", "POSITION_ERROR", "NOT_HOMED", "NOT_ACTIVE")

    def __init__(self):
        """Initializes ROS and necessary services and publishers.
        """

        rospy.init_node(
            "front_loader_planning"
            # disable_signals=True
            # log_level=rospy.DEBUG
        )

        self.front_loader_action_name = rospy.get_param("~front_loader_action_name", "front_loader_actions")

        self.front_loader_server = actionlib.SimpleActionServer(self.front_loader_action_name, FrontLoaderAction, self.front_loader_callback, auto_start=False)
        self.front_loader_server.start()
        rospy.loginfo("[%s] server started" % self.front_loader_action_name)

        self.result = FrontLoaderResult()

        self.move_pub = rospy.Publisher("linear_cmd", DodobotLinear, queue_size=100)
        self.parallel_gripper_pub = rospy.Publisher("parallel_gripper_cmd", DodobotParallelGripper, queue_size=100)

        self.linear_sub = rospy.Subscriber("linear", DodobotLinear, self.linear_callback, queue_size=100)
        self.parallel_gripper_sub = rospy.Subscriber("parallel_gripper", DodobotParallelGripper, self.parallel_gripper_callback, queue_size=100)
        self.events = Queue.Queue()

        self.is_goal_set = False

        self.has_error = False
        self.is_homed = False
        self.is_active = False

        self.linear_event_topic = "linear_events"
        self.linear_event_sub = rospy.Subscriber(self.linear_event_topic, DodobotLinearEvent, self.linear_event_callback, queue_size=100)
        self.linear_event_timeout = 20.0

        self.stepper_ticks_per_R_no_gearbox = rospy.get_param("~stepper_ticks_per_R_no_gearbox", 200.0)
        self.microsteps = rospy.get_param("~microsteps", 8.0)
        self.stepper_gearbox_ratio = rospy.get_param("~stepper_gearbox_ratio", 26.0 + 103.0 / 121.0)
        self.belt_pulley_radius_m = rospy.get_param("~belt_pulley_radius_m", 0.0121)

        self.stepper_ticks_per_R = self.stepper_ticks_per_R_no_gearbox * self.microsteps * self.stepper_gearbox_ratio
        self.stepper_R_per_tick = 1.0 / self.stepper_ticks_per_R
        self.step_ticks_to_linear_m = self.stepper_R_per_tick * self.belt_pulley_radius_m * 2 * math.pi
        self.step_linear_m_to_ticks = 1.0 / self.step_ticks_to_linear_m

        self.gripper_max_dist = 0.08
        self.gripper_dist = self.gripper_max_dist
        self.parallel_gripper_msg = DodobotParallelGripper()

        rospy.loginfo("[%s] --- Dodobot front loader planning is up! ---" % self.front_loader_action_name)

    ### CALLBACK FUNCTIONS ###

    def parallel_gripper_callback(self, msg):
        self.gripper_dist = gripper_msg.distance

    def linear_event_callback(self, msg):
        if self.is_goal_set:
            event_str = self.to_event_str(msg.event_num)
            self.events.put(event_str)

    def front_loader_callback(self, goal):
        move_cmd = goal.goal_z
        max_speed = goal.z_speed
        acceleration = goal.z_accel
        grip_dist = goal.grip_dist
        grip_threshold = goal.grip_threshold
        # transform pose from robot frame to front loader frame
        # pose_base_link = req.pose

        # perform IK to get command
        # move_cmd = self.inverse_kinematics(pose_base_link)
        if math.isnan(move_cmd) and math.isnan(grip_dist):
            rospy.loginfo("No action required from front loader. Skipping.")
            self.result.success = True
            self.front_loader_server.set_succeeded(self.result)
            return

        # send command
        if not math.isnan(grip_dist):
            success = self.send_grab_cmd(grip_dist, grip_threshold)

            if not success:
                self.result.success = success
                self.front_loader_server.set_succeeded(self.result)
                return

        if not math.isnan(move_cmd):
            success = self.send_move_cmd(move_cmd, max_speed, acceleration)

            if not success:
                self.result.success = success
                self.front_loader_server.set_succeeded(self.result)
                return

        # send response
        self.result.success = True
        self.front_loader_server.set_succeeded(self.result)


    ### HELPER FUNCTIONS ###

    def transform_pose(self, pose):
        """Transforms a pose from the base_link frame to the front_loader frame.

        Args:
            pose (Pose): Pose in the base_link frame

        Returns:
            (Pose): Pose in the front_loader frame.
        """

        # http://docs.ros.org/jade/api/tf/html/python/tf_python.html

        # initialize transformer
        t = tf.Transformer(True, rospy.Duration(10.0))

        # create temp transform
        m = TransformStamped()
        m.header.frame_id = 'base_link'
        m.child_frame_id = 'desired_front_loader_pos'
        m.transform.translation = pose.position
        m.transform.rotation = pose.orientation

        # lookup transform from existing tree
        t.setTransform(m)
        (pos, quat) = t.lookupTransform('front_loader', 'desired_front_loader_pos', rospy.Time(0))

        # return transformed pose
        pose_front_loader = Pose()
        pose_front_loader.position = pos
        pose_front_loader.orientation = quat
        return pose_front_loader

    def inverse_kinematics(self, pose):
        """Given a pose in the front_loader frame, determine the command to send the servo.

        Args:
            pose (Pose): The desired pose in the front_loader frame.

        Returns:
            (Integer): Corresponding servo command to send.
        """

        return pose.position.z

    def send_move_cmd(self, cmd, max_speed=None, acceleration=None):
        """Sends move command to self.move_pub. On completion, return true. On timeout, return false.

        Args:
            cmd (Integer): Servo command received from self.inverse_kinematics

        Returns:
            (Bool): True if command was successful. False otherwise.
        """

        if self.has_error:
            rospy.logerr("Can't send move command. Linear reports an error!")
            return False
        if not self.is_homed:
            rospy.logerr("Can't send move command. Linear is not homed!")
            return False
        if not self.is_active:
            rospy.logerr("Can't send move command. Linear is not active!")
            return False

        msg = DodobotLinear()
        msg.command_type = 0
        msg.command_value = int(cmd * self.step_linear_m_to_ticks)

        if math.isnan(max_speed):
            msg.max_speed = -1
        else:
            msg.max_speed = int(max_speed * self.step_linear_m_to_ticks)

        if math.isnan(acceleration):
            msg.acceleration = -1
        else:
            msg.acceleration = int(acceleration * self.step_linear_m_to_ticks)

        rospy.loginfo("Publishing command: %s\t%s\t%s" % (msg.command_value, msg.max_speed, msg.acceleration))

        self.move_pub.publish(msg)

        self.is_goal_set = True
        success = self.wait_for_linear()
        self.is_goal_set = False

        rospy.loginfo("Linear result: %s" % success)

        return success

    def send_grab_cmd(self, distance, force_threshold):
        self.parallel_gripper_msg.distance = distance
        self.parallel_gripper_msg.force_threshold = force_threshold
        self.parallel_gripper_pub.publish(self.parallel_gripper_msg.distance)

        return self.wait_for_gripper()

    def wait_for_gripper(self):
        return True

    def wait_for_linear(self):
        while True:
            if rospy.is_shutdown():
                return
            if self.front_loader_server.is_preempt_requested():
                rospy.loginfo("%s: Preempted" % self.front_loader_action_name)
                self.front_loader_server.set_preempted()
                return False

            try:
                event_str = self.events.get(timeout=self.linear_event_timeout)
            except Queue.Empty, rospy.ROSException:
                rospy.logerr("Move command timed out after %ss" % self.linear_event_timeout)
                return False
            # try:
            #     event_msg = rospy.wait_for_message(self.linear_event_topic, DodobotLinearEvent, timeout=self.linear_event_timeout)
            # except rospy.ROSException:
            #     rospy.logerr("Move command timed out after %ss" % self.linear_event_timeout)
            #     return False

            if event_str == "UNKNOWN":
                rospy.logerr("Unknown linear event type")
                continue

            elif event_str == "MOVE_FINISHED":
                rospy.loginfo("Move finished event received!")
                return True

            elif event_str == "MOVE_STARTED":
                rospy.loginfo("Move started")

            elif event_str in self.FAILED_GOAL_EVENTS:
                rospy.logerr("Move command failed: %s" % event_str)
                return False

    def linear_callback(self, msg):
        self.has_error = msg.has_error
        self.is_homed = msg.is_homed
        self.is_active = msg.is_active

    def to_event_str(self, event_num):
        if event_num in self.LINEAR_EVENTS:
            return self.LINEAR_EVENTS[event_num]
        else:
            return "UNKNOWN"


if __name__ == "__main__":
    try:
        node = FrontLoaderPlanner()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass

    finally:
        rospy.loginfo("Exiting front_loader_planning node")
