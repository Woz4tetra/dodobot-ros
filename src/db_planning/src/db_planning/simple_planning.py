#!/usr/bin/env python3
import rospy
import actionlib
from std_srvs.srv import Trigger, TriggerResponse

from db_planning.srv import NamedPose, NamedPoseResponse
from db_planning.msg import SequenceRequestAction, SequenceRequestGoal, SequenceRequestResult


class SimplePlanning:
    def __init__(self):
        self.node_name = "simple_planning"
        rospy.init_node(
            self.node_name
            # disable_signals=True
            # log_level=rospy.DEBUG
        )
        self.dock_request_srv = rospy.Service("dock_request", Trigger, self.dock_request_callback)
        self.park_request_srv = rospy.Service("park_request", NamedPose, self.park_request_callback)
        self.undock_request_srv = rospy.Service("undock_request", Trigger, self.undock_request_callback)

        self.central_action = actionlib.SimpleActionClient("sequence_request", SequenceRequestAction)
        self.central_action.wait_for_server()

    def park_request_callback(self, req):
        goal = SequenceRequestGoal()
        goal.type = SequenceRequestGoal.NAMED_GOAL
        goal.action = SequenceRequestGoal.PARK
        goal.name = req.name
        self.central_action.send_goal(goal)
        self.central_action.wait_for_result()
        result = self.central_action.get_result()
        return NamedPoseResponse(result.success)

    def dock_request_callback(self, req):
        goal = SequenceRequestGoal()
        goal.type = SequenceRequestGoal.NAMED_GOAL
        goal.action = SequenceRequestGoal.DOCK
        goal.name = "dock"
        self.central_action.send_goal(goal)
        self.central_action.wait_for_result()
        result = self.central_action.get_result()
        return TriggerResponse(result.success, "")
        
    def undock_request_callback(self, req):
        goal = SequenceRequestGoal()
        goal.type = SequenceRequestGoal.NAMED_GOAL
        goal.action = SequenceRequestGoal.UNDOCK
        goal.name = "dock"
        self.central_action.send_goal(goal)
        self.central_action.wait_for_result()
        result = self.central_action.get_result()
        return TriggerResponse(result.success, "")
        
    def run(self):
        rospy.spin()

if __name__ == "__main__":
    node = SimplePlanning()
    try:
        node.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        rospy.loginfo("Exiting %s node" % (node.node_name))
