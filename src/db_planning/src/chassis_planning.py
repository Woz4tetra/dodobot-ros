#!/usr/bin/python

import tf
import rospy


class ChassisPlanning:
    """
    Class definition for planning_base ROS node.
    """

    def __init__(self):
        """
        Initializes ROS and necessary services and publishers.
        """

        rospy.init_node(
            "chassis_planning"
            # disable_signals=True
            # log_level=rospy.DEBUG
        )

if __name__ == "__main__":
    try:
        node = ChassisPlanning()
        node.run()

    except rospy.ROSInterruptException:
        pass

    finally:
        rospy.loginfo("Exiting planning_base node")
