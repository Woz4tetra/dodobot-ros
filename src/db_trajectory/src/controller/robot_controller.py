from state import Pose2d, Velocity


class RobotController:
    def __init__(self):
        self.current_pose = Pose2d()
        self.goal_pose = Pose2d()
        self.pose_tolerance = Pose2d()
        self.enabled = True

        self.lower_v = Velocity(None, None, None)
        self.upper_v = Velocity(None, None, None)
    
    def set_tolerance(self, pose_tolerance):
        self.pose_tolerance = pose_tolerance

    def set_goal(self, goal_pose):
        self.goal_pose = goal_pose

    def set_bounds(self, lower, upper):
        self.lower_v = lower
        self.upper_v = upper

    def arrived(self):
        pose_error = self.goal_pose - self.current_pose
        return abs(pose_error) < self.pose_tolerance

    def update(self, current_pose, desired_v):
        raise NotImplementedError
