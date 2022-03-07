#!/usr/bin/env python3
import rospy
import rostopic

from dynamic_reconfigure.client import Client as DynamicClient


class DodobotCameraWatcher:
    def __init__(self):
        self.node_name = "db_camera_watcher"
        rospy.init_node(
            self.node_name
            # disable_signals=True
            # log_level=rospy.DEBUG
        )
        # rospy.on_shutdown(self.shutdown_hook)

        self.camera_ns = rospy.get_param("~camera_ns", "/camera")
        self.expected_camera_rate = rospy.get_param("~expected_camera_rate", 15.0)
        self.expected_depth_rate = rospy.get_param("~expected_depth_rate", 15.0)
        self.min_rate_offset = rospy.get_param("~rate_band", 5.0)
        self.color_min_rate_threshold = max(0.0, self.expected_camera_rate - self.min_rate_offset)
        self.color_max_rate_threshold = max(0.0, self.expected_camera_rate + self.min_rate_offset)
        self.depth_min_rate_threshold = max(0.0, self.expected_depth_rate - self.min_rate_offset)
        self.depth_max_rate_threshold = max(0.0, self.expected_depth_rate + self.min_rate_offset)
        self.l500_depth_config = rospy.get_param("~l500_depth_config", None)
        self.motion_module_config = rospy.get_param("~motion_module_config", None)
        self.rgb_camera_config = rospy.get_param("~rgb_camera_config", None)

        self.camera_rate = rostopic.ROSTopicHz(15)
        self.camera_topic = self.camera_ns + "/color/camera_info"
        rospy.Subscriber(self.camera_topic, rospy.AnyMsg, self.camera_rate.callback_hz, callback_args=self.camera_topic, queue_size=1)

        self.depth_rate = rostopic.ROSTopicHz(15)
        self.depth_topic = self.camera_ns + "/depth/camera_info"
        rospy.Subscriber(self.depth_topic, rospy.AnyMsg, self.depth_rate.callback_hz, callback_args=self.depth_topic, queue_size=1)

        self.l500_depth_client = None
        self.motion_module_client = None
        self.rgb_camera_client = None
        self.l500_depth_dyn_topic = self.camera_ns + "/l500_depth_sensor"
        self.motion_module_dyn_topic = self.camera_ns + "/motion_module"
        self.rgb_camera_dyn_topic = self.camera_ns + "/rgb_camera"

        rospy.loginfo("%s init complete" % self.node_name)

    def init_dynamic_clients(self):
        if self.l500_depth_client is None:
            self.l500_depth_client = DynamicClient(self.l500_depth_dyn_topic)
        if self.motion_module_client is None:
            self.motion_module_client = DynamicClient(self.motion_module_dyn_topic)
        if self.rgb_camera_client is None:
            self.rgb_camera_client = DynamicClient(self.rgb_camera_dyn_topic)

    def get_publish_rate(self):
        color_result = self.camera_rate.get_hz(self.camera_topic)
        depth_result = self.depth_rate.get_hz(self.depth_topic)
        color_rate = 0.0 if color_result is None else color_result[0]
        depth_rate = 0.0 if depth_result is None else depth_result[0]
        return color_rate, depth_rate

    def set_camera_parameters(self):
        self.init_dynamic_clients()
        if self.l500_depth_config is not None:
            rospy.loginfo("Updating l500_depth parameters: %s" % str(self.l500_depth_config))
            rospy.wait_for_service(self.l500_depth_dyn_topic + "/set_parameters", 30.0)
            self.l500_depth_client.update_configuration(self.l500_depth_config)
        else:
            rospy.loginfo("l500_depth parameters are not set. Skipping dynamic reconfigure")
        if self.motion_module_config is not None:
            rospy.loginfo("Updating motion_module parameters: %s" % str(self.motion_module_config))
            rospy.wait_for_service(self.motion_module_dyn_topic + "/set_parameters", 30.0)
            self.motion_module_client.update_configuration(self.motion_module_config)
        else:
            rospy.loginfo("motion_module parameters are not set. Skipping dynamic reconfigure")
        if self.rgb_camera_config is not None:
            rospy.loginfo("Updating rgb_camera parameters: %s" % str(self.rgb_camera_config))
            rospy.wait_for_service(self.rgb_camera_dyn_topic + "/set_parameters", 30.0)
            self.rgb_camera_client.update_configuration(self.rgb_camera_config)
        else:
            rospy.loginfo("rgb_camera parameters are not set. Skipping dynamic reconfigure")

    def run(self):
        parameters_set = False
        while not rospy.is_shutdown():
            rospy.sleep(1.0)
            color_rate, depth_rate = self.get_publish_rate()
            if not parameters_set and (color_rate > 0.0 or depth_rate > 0.0):
                self.set_camera_parameters()
                parameters_set = True

            if not (self.color_min_rate_threshold <= color_rate <= self.color_max_rate_threshold and
                    self.depth_min_rate_threshold <= depth_rate <= self.depth_max_rate_threshold):
                rospy.logwarn_throttle(2.0, 
                    "Camera is not publishing within the threshold. " \
                    "Color (%0.1f..%0.1f): %0.2f. Depth (%0.1f..%0.1f): %0.2f" % (
                        self.color_min_rate_threshold, self.color_max_rate_threshold, color_rate,
                        self.depth_min_rate_threshold, self.depth_max_rate_threshold, depth_rate)
                )


if __name__ == "__main__":
    node = DodobotCameraWatcher()
    try:
        node.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        rospy.loginfo("Exiting %s node" % node.node_name)
