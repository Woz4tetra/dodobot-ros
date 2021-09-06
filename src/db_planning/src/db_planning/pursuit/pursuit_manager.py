import rospy
from dodobot_tools.robot_state import Pose2d
from dodobot_tools.recursive_namespace import RecursiveNamespace
from .drive_towards import DriveTowards
from .turn_towards import TurnTowards
from .turn_final import TurnFinal


class PursuitManager:
    def __init__(self, pursuit_parameters: RecursiveNamespace, set_velocity, get_state, should_stop):
        self.pursuit_parameters = pursuit_parameters
        self.goal_pose = Pose2d()
        self.set_velocity = set_velocity  # function with two parameters: linear_x, ang_v
        self.get_state = get_state  # function that returns Pose2d
        self.should_stop = should_stop  # function that returns True when the pursuit should stop

        self.drive_towards = DriveTowards(self.pursuit_parameters, set_velocity, get_state, should_stop)
        self.turn_towards = TurnTowards(self.pursuit_parameters, set_velocity, get_state, should_stop)
        self.turn_final = TurnFinal(self.pursuit_parameters, set_velocity, get_state, should_stop)

    def set_parameters(self, parameters):
        self.pursuit_parameters.update(parameters)
        self.drive_towards.parameters.update(parameters)
        self.turn_towards.parameters.update(parameters)
        self.turn_final.parameters.update(parameters)

    def run(self):
        self.stop_motors()
        result_state = None

        # assumes self.goal_pose is set
        if not self.pursuit_parameters.turn_in_place_only:
            while True:
                rospy.loginfo("Turning towards goal")
                result_state = self.turn_towards.run()
                if result_state != "success":
                    break
                rospy.loginfo("Driving towards goal")
                result_state = self.drive_towards.run()
                if result_state != "turn":
                    break
        if self.pursuit_parameters.turn_in_place_only or (self.pursuit_parameters.turn_towards_final_heading and result_state == "success"):
            rospy.loginfo("Turning towards final heading")
            result_state = self.turn_final.run()
        return result_state

    def stop_motors(self):
        self.set_velocity(0.0, 0.0)

    def set_goal(self, x, y, theta):
        self.goal_pose.x = x
        self.goal_pose.y = y
        self.goal_pose.theta = theta
        self.drive_towards.set_goal(self.goal_pose)
        self.turn_towards.set_goal(self.goal_pose)
        self.turn_final.set_goal(self.goal_pose)
