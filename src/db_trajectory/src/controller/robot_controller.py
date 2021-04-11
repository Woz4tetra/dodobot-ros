from .robot_state import Pose2d


class RobotController:
    def __init__(self):
        self.current_pose = Pose2d()
        self.goal_pose = Pose2d()
        self.pose_tolerance = Pose2d()
        self.enabled = True
    
    def set_tolerance(self, x, y, theta):
        self.pose_tolerance = Pose2d.from_xyt(x, y, theta)

    def arrived(self):
        pose_error = self.goal_pose - self.current_pose
        return abs(pose_error) < self.pose_tolerance

    def set_goal(self, goal_pose):
        self.goal_pose = goal_pose

    def update(self, current_pose, desired_v):
        raise NotImplementedError
