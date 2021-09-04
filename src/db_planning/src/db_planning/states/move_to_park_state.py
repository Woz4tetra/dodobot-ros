import rospy
from smach import State
from actionlib_msgs.msg import GoalStatus

from db_planning.msg import SequenceRequestGoal


class MoveToParkState(State):
    def __init__(self, central_planning):
        super(MoveToParkState, self).__init__(
            outcomes=["success", "preempted", "failure"],
            input_keys=["sequence_goal", "central_planning"],
        )
        self.central_planning = central_planning
        self.check_state_interval = 0.25  # seconds
    
    def execute(self, userdata):
        goal = userdata.sequence_goal
        if goal.type == SequenceRequestGoal.NAMED_GOAL:
            pass
        elif goal.type == SequenceRequestGoal.POSE_GOAL:
            raise NotImplementedError

        goal_pose = self.central_planning.get_goal_pose(goal)

        self.central_planning.set_planner_velocities(
            max_vel_x=self.central_planning.max_vel_x,
            max_vel_theta=self.central_planning.max_vel_theta,
        )
        if goal_pose is None:
            return "failure"
        rospy.loginfo("Start goal: %0.4f, %0.4f, %0.4f" % (goal_pose.pose.position.x, goal_pose.pose.position.y, goal_pose.pose.position.z))

        self.central_planning.set_move_base_goal(goal_pose)
        self.central_planning.look_straight_ahead()
        
        while True:
            if rospy.is_shutdown():
                return "preempted"
            
            state = self.central_planning.get_move_base_state()
            if state == GoalStatus.SUCCEEDED:
                return "success"
            elif state == GoalStatus.ABORTED:
                return "preempted"

            rospy.sleep(self.check_state_interval)
