#!/usr/bin/env python3
import rospy
import rostopic

from dynamic_reconfigure.client import Client as DynamicClient


class DodobotLidarWatcher:
    def __init__(self):
        self.node_name = "db_lidar_watcher"
        rospy.init_node(
            self.node_name
            # disable_signals=True
            # log_level=rospy.DEBUG
        )
        # rospy.on_shutdown(self.shutdown_hook)

        self.expected_left_rate = rospy.get_param("~expected_left_rate", 10.0)
        self.expected_right_rate = rospy.get_param("~expected_right_rate", 10.0)
        self.min_rate_offset = rospy.get_param("~rate_band", 5.0)
        self.left_min_rate_threshold = max(0.0, self.expected_left_rate - self.min_rate_offset)
        self.left_max_rate_threshold = max(0.0, self.expected_left_rate + self.min_rate_offset)
        self.right_min_rate_threshold = max(0.0, self.expected_right_rate - self.min_rate_offset)
        self.right_max_rate_threshold = max(0.0, self.expected_right_rate + self.min_rate_offset)
        self.combiner_config = rospy.get_param("~combiner_config", None)

        self.left_rate = rostopic.ROSTopicHz(15)
        self.left_topic = "/laser/scan_filtered"
        rospy.Subscriber(self.left_topic, rospy.AnyMsg, self.left_rate.callback_hz, callback_args=self.left_topic, queue_size=5)

        self.right_rate = rostopic.ROSTopicHz(15)
        self.right_topic = "/test_laser/scan_filtered"
        rospy.Subscriber(self.right_topic, rospy.AnyMsg, self.right_rate.callback_hz, callback_args=self.right_topic, queue_size=5)

        self.combiner_client = None
        self.combiner_dyn_topic = "/laserscan_multi_merger"

        rospy.loginfo("%s init complete" % self.node_name)

    def init_dynamic_clients(self):
        if self.combiner_client is None:
            self.combiner_client = DynamicClient(self.combiner_dyn_topic)

    def get_publish_rate(self):
        left_result = self.left_rate.get_hz(self.left_topic)
        right_result = self.right_rate.get_hz(self.right_topic)
        left_rate = 0.0 if left_result is None else left_result[0]
        right_rate = 0.0 if right_result is None else right_result[0]
        return left_rate, right_rate

    def set_parameters(self):
        self.init_dynamic_clients()
        if self.combiner_config is not None:
            rospy.loginfo("Updating laser combiner parameters: %s" % str(self.combiner_config))
            rospy.wait_for_service(self.combiner_dyn_topic + "/set_parameters", 30.0)
            self.combiner_client.update_configuration(self.combiner_config)
        else:
            rospy.loginfo("laser combiner parameters are set. Skipping dynamic reconfigure")

    def run(self):
        parameters_set = False
        while not rospy.is_shutdown():
            rospy.sleep(1.0)
            left_rate, right_rate = self.get_publish_rate()
            if not parameters_set and (left_rate > 0.0 or right_rate > 0.0):
                self.set_parameters()
                parameters_set = True

            if not (self.left_min_rate_threshold <= left_rate <= self.left_max_rate_threshold and
                    self.right_min_rate_threshold <= right_rate <= self.right_max_rate_threshold):
                rospy.logwarn_throttle(2.0, 
                    "LIDARs are not publishing within the threshold. " \
                    "left (%0.1f..%0.1f): %0.2f. right (%0.1f..%0.1f): %0.2f" % (
                        self.left_min_rate_threshold, self.left_max_rate_threshold, left_rate,
                        self.right_min_rate_threshold, self.right_max_rate_threshold, right_rate)
                )


if __name__ == "__main__":
    node = DodobotLidarWatcher()
    try:
        node.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        rospy.loginfo("Exiting %s node" % node.node_name)
