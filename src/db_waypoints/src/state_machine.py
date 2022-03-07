import rospy

from smach import StateMachine

from states.go_to_waypoint import GoToWaypointState
from states.check_waypoint import CheckWaypointState
from states.next_waypoint import NextWaypointState
from states.pursue_object import PursueObjectState


class WaypointStateMachine(object):
    def __init__(self):
        self.sm = StateMachine(outcomes=["success", "failure", "preempted"])
        self.outcome = None
        self.waypoints_node = None
        self.action_goal = None

        with self.sm:
            StateMachine.add(
                "CHECK_WAYPOINT", CheckWaypointState(self),
                transitions={
                    "waypoint": "GOTO_WAYPOINT",
                    "object": "PURSUE_OBJECT",
                    "finished": "success",
                    "failure": "failure",
                    "preempted": "preempted"
                },
                remapping={
                    "waypoints_plan": "sm_waypoints_plan",
                    "waypoint_index": "sm_waypoint_index",
                }
            )
            StateMachine.add(
                "GOTO_WAYPOINT", GoToWaypointState(self),
                transitions={
                    "object": "PURSUE_OBJECT",
                    "success": "NEXT_WAYPOINT",
                    "failure": "failure",
                    "preempted": "preempted"
                },
                remapping={
                    "waypoints_plan": "sm_waypoints_plan",
                    "waypoint_index": "sm_waypoint_index",
                }
            )
            StateMachine.add(
                "PURSUE_OBJECT", PursueObjectState(self),
                transitions={
                    "success": "NEXT_WAYPOINT",
                    "failure": "failure",
                    "preempted": "preempted"
                },
                remapping={
                    "waypoints_plan": "sm_waypoints_plan",
                    "waypoint_index": "sm_waypoint_index",
                }
            )
            StateMachine.add(
                "NEXT_WAYPOINT", NextWaypointState(),
                transitions={
                    "success": "CHECK_WAYPOINT",
                    "finished": "success",
                    "failure": "failure",
                    "preempted": "preempted"
                },
                remapping={
                    "waypoint_index_in": "sm_waypoint_index",
                    "waypoints_plan": "sm_waypoints_plan",
                    "waypoint_index_out": "sm_waypoint_index",
                }
            )

    def close(self):
        if self.waypoints_node is None:
            return
        self.waypoints_node.move_base.cancel_goal()
        self.waypoints_node.toggle_local_costmap(True)
        self.waypoints_node.toggle_walls(True)

    def execute(self, waypoints_plan, waypoints_node):
        rospy.loginfo("To cancel the waypoint follower, run: 'rostopic pub -1 /dodobot/follow_path/cancel actionlib_msgs/GoalID -- {}'")
        rospy.loginfo("To cancel the current goal, run: 'rostopic pub -1 /move_base/cancel actionlib_msgs/GoalID -- {}'")

        self.waypoints_node = waypoints_node

        self.sm.userdata.sm_waypoint_index = 0
        self.sm.userdata.sm_waypoints_plan = waypoints_plan
        self.outcome = self.sm.execute()
        if self.outcome == "success":
            self.waypoints_node.follow_path_server.set_succeeded()
        if self.outcome == "failure":
            self.waypoints_node.follow_path_server.set_aborted()
        if self.outcome == "preempted":
            self.waypoints_node.follow_path_server.set_preempted()

        self.waypoints_node.move_base.cancel_goal()
        return self.outcome
