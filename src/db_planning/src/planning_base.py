#!/usr/bin/python

import rospy
import actionlib

from move_base.msg import MoveBaseAction, MoveBaseGoal

from db_planning.srv import BaseMove, BaseMoveResponse

class PlanningBase:
    """Class definition for planning_base ROS node.
    """

    def __init__(self):
        """Initializes ROS and necessary services and publishers.
        """

        rospy.init_node(
            "planning_base"
            # disable_signals=True
            # log_level=rospy.DEBUG
        )

        # create base_move service        
        self.move_service = rospy.Service('base_move', BaseMove, self.move_callback)

        # wait for action client 
        self.move_action_client = actionlib.SimpleActionClient('move_base/goal', MoveBaseAction)
        self.move_action_client.wait_for_server()

    ### CALLBACK FUNCTIONS ###

    def move_callback(self, req):
        """Service callback for base_move. Moves robot to the desired pose.
        Args:
            req (BaseMove): BaseMove request object
        Returns:
            (BaseMoveResponse): Response object for indicating (un)successful completion of action.
        """

        # transform pose from robot frame to front loader frame
        pose_base_link = req.pose

        # Creates a goal to send to the action server.
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.pose = pose_base_link
    
        # Sends the goal to the action server.
        self.move_action_client.send_goal(goal)
    
        # Waits for the server to finish performing the action.
        self.move_action_client.wait_for_result()
    
        # Get the result of executing the action
        # result = self.move_action_client.get_result()

        return BaseMoveResponse(True)

if __name__ == "__main__":
    try:
        node = PlanningBase()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass

    finally:
        rospy.loginfo("Exiting planning_base node")