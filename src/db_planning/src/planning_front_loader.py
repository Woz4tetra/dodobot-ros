#!/usr/bin/python
import rospy
import tf

from geometry_msgs.msg import Pose, TransformStamped

from db_planning.srv import FrontLoaderMove, FrontLoaderMoveResponse
from db_planning.srv import FrontLoaderGrab, FrontLoaderGrabResponse
from db_planning.srv import FrontLoaderPlan, FrontLoaderPlanResponse

from db_parsing.msg import DodobotLinear, DodobotGripper


class PlanningFrontLoader:
    """Class definition for planning_front_loader ROS node.
    """

    def __init__(self):
        """Initializes ROS and necessary services and publishers.
        """

        rospy.init_node(
            "planning_front_loader"
            # disable_signals=True
            # log_level=rospy.DEBUG
        )

        self.move_service = rospy.Service('front_loader_move', FrontLoaderMove, self.move_callback)
        self.grab_service = rospy.Service('front_loader_grab', FrontLoaderGrab, self.grab_callback)
        self.plan_service = rospy.Service('front_loader_plan', FrontLoaderPlan, self.plan_callback)

        self.move_pub = rospy.Publisher("linear_cmd", DodobotLinear, queue_size=100)
        self.grab_pub = rospy.Publisher("gripper_cmd", DodobotGripper, queue_size=100)

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

        return True

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
        node = PlanningFrontLoader()
        rospy.spin()
        
    except rospy.ROSInterruptException:
        pass

    finally:
        rospy.loginfo("Exiting planning_front_loader node")

