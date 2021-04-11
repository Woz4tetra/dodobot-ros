import rospy

from tf.transformations import euler_from_quaternion

from nav_msgs.msg import Odometry

from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped

from controller import RamseteController, Pose2d, Velocity


class DodobotTrajectory:
    def __init__(self):
        self.node_name = "db_trajectory"
        rospy.init_node(
            self.node_name
            # disable_signals=True
            # log_level=rospy.DEBUG
        )

        self.ramsete_b = rospy.get_param("~ramsete_b", 2.0)
        self.ramsete_zeta = rospy.get_param("~ramsete_b", 0.7)

        self.odom_topic = rospy.get_param("~odom_topic", "odom")
        self.cmd_vel_topic = rospy.get_param("~cmd_vel_topic", "cmd_vel")

        self.controller = RamseteController(self.ramsete_b, self.ramsete_zeta)
    
        self.odom_sub = rospy.Subscriber(self.odom_topic, Odometry, self.odom_callback, queue_size=5)
        self.goal_sub = rospy.Subscriber("controller_simple_goal", PoseStamped, self.goal_callback, queue_size=5)
        
        self.cmd_vel_pub = rospy.Publisher(self.cmd_vel_topic, Twist, queue_size=5)
    
    def goal_callback(self, msg):
        self.desired_v = Velocity()  # for singular goal point, desired end velocity is zero
        goal_pose = self.to_pose2d(msg.pose)
        self.controller.set_goal(goal_pose)

    def odom_callback(self, msg):
        if self.controller.arrived():
            return
        
        current_pose = self.to_pose2d(msg.pose.pose)
        chassis_speeds = self.controller.update(current_pose, self.desired_v)

        twist = Twist()
        twist.linear.x = chassis_speeds.x
        twist.angular.z = chassis_speeds.theta
        self.cmd_vel_pub.publish(twist)
    
    def to_pose2d(self, pose):
        theta = euler_from_quaternion((
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        ))
        return Pose2d.from_xyt(
            pose.position.x,
            pose.position.y,
            theta
        )

    def run(self):
        rospy.spin()


def main():
    node = DodobotTrajectory()
    try:
        node.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        rospy.loginfo("Exiting %s node" % node.node_name)

if __name__ == "__main__":
    main()
