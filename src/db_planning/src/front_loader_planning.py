#!/usr/bin/python

import rospy
import tf
import math

from geometry_msgs.msg import Pose, TransformStamped

from db_planning.srv import FrontLoaderMove, FrontLoaderMoveResponse
from db_planning.srv import FrontLoaderGrab, FrontLoaderGrabResponse
from db_planning.srv import FrontLoaderPlan, FrontLoaderPlanResponse

from db_parsing.msg import DodobotLinear, DodobotLinearEvent, DodobotGripper


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

    FAILED_GOAL_EVENTS = ("ACTIVE_TRUE", "ACTIVE_FALSE", "HOMING_STARTED", "HOMING_FINISHED", "POSITION_ERROR", "NOT_HOMED", "NOT_ACTIVE")

    def __init__(self):
        """Initializes ROS and necessary services and publishers.
        """

        rospy.init_node(
            "front_loader_planning"
            # disable_signals=True
            # log_level=rospy.DEBUG
        )

        self.move_service = rospy.Service('front_loader_move', FrontLoaderMove, self.move_callback)
        self.grab_service = rospy.Service('front_loader_grab', FrontLoaderGrab, self.grab_callback)
        self.plan_service = rospy.Service('front_loader_plan', FrontLoaderPlan, self.plan_callback)

        self.move_pub = rospy.Publisher("linear_cmd", DodobotLinear, queue_size=100)
        self.grab_pub = rospy.Publisher("gripper_cmd", DodobotGripper, queue_size=100)

        self.linear_sub = rospy.Subscriber("linear", DodobotLinear, self.linear_callback, queue_size=100)

        self.has_error = False
        self.is_homed = False
        self.is_active = False

        self.linear_event_topic = "linear_events"
        self.linear_event_timeout = 20.0

        self.stepper_ticks_per_R_no_gearbox = rospy.get_param("~stepper_ticks_per_R_no_gearbox", 200.0)
        self.microsteps = rospy.get_param("~microsteps", 8.0)
        self.stepper_gearbox_ratio = rospy.get_param("~stepper_gearbox_ratio", 26.0 + 103.0 / 121.0)
        self.belt_pulley_radius_m = rospy.get_param("~belt_pulley_radius_m", 0.0121)

        self.stepper_ticks_per_R = self.stepper_ticks_per_R_no_gearbox * self.microsteps * self.stepper_gearbox_ratio
        self.stepper_R_per_tick = 1.0 / self.stepper_ticks_per_R
        self.step_ticks_to_linear_m = self.stepper_R_per_tick * self.belt_pulley_radius_m * 2 * math.pi
        self.step_linear_m_to_ticks = 1.0 / self.step_ticks_to_linear_m

    ### CALLBACK FUNCTIONS ###

    def move_callback(self, req):
        """Service callback for front_loader_move. Moves servo into desired front loader pose.

        Args:
            req (FrontLoaderMove): FrontLoaderMove request object

        Returns:
            (FrontLoaderMoveResponse): Response object for indicating (un)successful completion of action.
        """

        # transform pose from robot frame to front loader frame
        pose_base_link = req.pose

        # perform IK to get command
        move_cmd = self.inverse_kinematics(pose_base_link)

        # send command
        success = self.send_move_cmd(move_cmd)

        # send response
        return FrontLoaderMoveResponse(success)

    def grab_callback(self, req):
        """Service callback for front_loader_grab. Opens or closes the gripper.

        Args:
            req (FrontLoaderGrab): FrontLoaderGrab request object

        Returns:
            (FrontLoaderGrabResponse): Response object for indicating (un)successful completion of action.
        """

        # send command
        success = self.send_grab_cmd(req.grab)

        # send response
        return FrontLoaderGrabResponse(success)

    def plan_callback(self, req):
        """Service callback for front_loader_plan.

        Args:
            req (FrontLoaderPlan): FrontLoaderPlan request object

        Returns:
            (FrontLoaderPlanResponse): Response object for indicating (un)successful completion of action.
        """

        # call move_callback
        move_req = FrontLoaderMove()
        move_req.pose = req.pose
        success = self.move_callback(move_req)
        if (success == False): return FrontLoaderPlanResponse(False)

        # call grab_callback
        grab_req = FrontLoaderGrab()
        grab_req.grab = req.grab
        success = self.grab_callback(grab_req)
        if (success == False): return FrontLoaderPlanResponse(False)

        # send response
        return FrontLoaderPlanResponse(True)

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

    def send_move_cmd(self, cmd):
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
        self.move_pub.publish(msg)

        rospy.loginfo("Publishing command: %s" % msg.command_value)

        while True:
            try:
                event_msg = rospy.wait_for_message(self.linear_event_topic, DodobotLinearEvent, timeout=self.linear_event_timeout)
            except rospy.ROSException:
                rospy.logerr("Move command timed out after %ss" % self.linear_event_timeout)
                return False

            event_str = self.to_event_str(event_msg.event_num)
            if event_str == "UNKNOWN":
                rospy.logerr("Unknown linear event type: %s" % event_msg.event_num)
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

    def send_grab_cmd(self, cmd):
        """Sends grab command to self.grab_pub. On completion, return true. On timeout, return false.

        Args:
            cmd (Bool): True if gripper should close. False if gripper should open.

        Returns:
            (Bool): True if command was successful. False otherwise.
        """

        return True


if __name__ == "__main__":
    try:
        node = FrontLoaderPlanner()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass

    finally:
        rospy.loginfo("Exiting front_loader_planning node")
