#!/usr/bin/env python3

import math
import queue

import rospy
import actionlib

from geometry_msgs.msg import Pose, TransformStamped

from std_srvs.srv import Trigger, TriggerResponse

from db_planning.msg import FrontLoaderAction, FrontLoaderGoal, FrontLoaderResult
from db_parsing.msg import DodobotLinear, DodobotLinearEvent
from db_chassis.msg import LinearPosition

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

        self.result = FrontLoaderResult()

        self.move_pub = rospy.Publisher("linear_pos_cmd", LinearPosition, queue_size=100)
        self.events = queue.Queue()

        self.is_goal_set = False

        self.linear_event_topic = "linear_events"
        self.linear_event_sub = rospy.Subscriber(self.linear_event_topic, DodobotLinearEvent, self.linear_event_callback, queue_size=100)
        self.linear_event_timeout = 10.0

        self.linear_topic = "linear"
        self.linear_sub = rospy.Subscriber(self.linear_topic, DodobotLinear, self.linear_callback, queue_size=100)

        self.is_homed = False
        self.is_active = False
        self.has_error = False

        self.front_loader_ready_service_name = "front_loader_ready_service"
        rospy.loginfo("Setting up service %s" % self.front_loader_ready_service_name)
        self.front_loader_ready_srv = rospy.Service(self.front_loader_ready_service_name, Trigger, self.front_loader_ready_callback)
        rospy.loginfo("%s service is ready" % self.front_loader_ready_service_name)

        self.front_loader_server = actionlib.SimpleActionServer(self.front_loader_action_name, FrontLoaderAction, self.front_loader_callback, auto_start=False)
        self.front_loader_server.start()
        rospy.loginfo("[%s] server started" % self.front_loader_action_name)

        rospy.loginfo("[%s] --- Dodobot front loader planning is up! ---" % self.front_loader_action_name)

    ### CALLBACK FUNCTIONS ###

    def front_loader_ready_callback(self, req):
        if self.is_homed and self.is_active and not self.has_error:
            return TriggerResponse(True, "Ready!")
        else:
            return TriggerResponse(False, "homed: %s, active %s, has_error: %s" % (self.is_homed, self.is_active, self.has_error))

    def linear_event_callback(self, msg):
        if self.is_goal_set:
            event_str = self.to_event_str(msg.event_num)
            self.events.put(event_str)

    def linear_callback(self, msg):
        self.is_homed = msg.is_homed
        self.is_active = msg.is_active
        self.has_error = msg.has_error

    def front_loader_callback(self, goal):
        move_cmd = goal.goal_z
        max_speed = goal.z_speed
        acceleration = goal.z_accel

        wait_to_finish = not math.isnan(move_cmd)
        # if math.isnan(move_cmd):
        #     rospy.loginfo("No action required from front loader. Skipping.")
        #     self.result.success = True
        #     self.front_loader_server.set_succeeded(self.result)
        #     return

        success = self.send_move_cmd(move_cmd, max_speed, acceleration, wait_to_finish)

        # send response
        self.result.success = success
        self.front_loader_server.set_succeeded(self.result)

    def send_move_cmd(self, cmd, max_speed, acceleration, wait_to_finish):
        """Sends move command to self.move_pub. On completion, return true. On timeout, return false.

        Args:

        Returns:
            (Bool): True if command was successful. False otherwise.
        """

        msg = LinearPosition()
        msg.position = cmd
        msg.max_speed = max_speed
        msg.acceleration = acceleration

        rospy.loginfo("Publishing front loader command: %s\t%s\t%s" % (msg.position, msg.max_speed, msg.acceleration))

        self.move_pub.publish(msg)

        self.is_goal_set = True
        if wait_to_finish:
            success = self.wait_for_linear()
        else:
            success = True
        self.is_goal_set = False

        rospy.loginfo("Linear result: %s" % success)

        return success

    def wait_for_linear(self):
        rate = rospy.Rate(15.0)
        while True:
            if rospy.is_shutdown() or self.front_loader_server.is_preempt_requested():
                rospy.loginfo("%s: Preempted" % self.front_loader_action_name)
                self.front_loader_server.set_preempted()
                return False

            try:
                event_str = self.events.get(timeout=self.linear_event_timeout)
            except (queue.Empty, rospy.ROSException) as e:
                rospy.logerr("Move command timed out after %ss: %s" % (self.linear_event_timeout, e))
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
            rate.sleep()

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
