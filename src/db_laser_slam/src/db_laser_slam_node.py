#!/usr/bin/env python3
import os
import rospy
import rospkg
import rostopic
from datetime import datetime
from collections import defaultdict

from db_laser_slam.srv import SetSlamMode, SetSlamModeResponse
from db_laser_slam.srv import GetSlamMode, GetSlamModeResponse

from std_srvs.srv import Trigger, TriggerResponse

from dodobot_tools.launch_manager import LaunchManager


class DodobotLaserSlam:
    def __init__(self):
        self.node_name = "db_laser_slam"
        rospy.init_node(
            self.node_name
            # disable_signals=True
            # log_level=rospy.DEBUG
        )
        rospy.on_shutdown(self.shutdown_hook)

        self.MAPPING = "mapping"
        self.LOCALIZE = "localize"
        self.IDLE = "idle"
        self.NONE = "none"
        self.MODES = [
            self.MAPPING,
            self.LOCALIZE,
            self.IDLE
        ]

        self.rospack = rospkg.RosPack()
        self.package_dir = self.rospack.get_path(self.node_name)
        self.default_maps_dir = self.package_dir + "/maps"
        self.default_launches_dir = self.package_dir + "/launch/sublaunch"

        self.service_ns_name = rospy.get_param("~service_ns_name", "/dodobot")
        self.start_mode = rospy.get_param("~mode", "mapping")
        self.min_localize_rate_threshold = rospy.get_param("~min_localize_rate_threshold", 1.0)
        self.min_mapping_rate_threshold = rospy.get_param("~min_mapping_rate_threshold", 1.0)
        
        self.mode = self.NONE
        self.map_dir = rospy.get_param("~map_dir", self.default_maps_dir)
        self.map_name = rospy.get_param("~map_name", "map-{date}")
        self.date_format = rospy.get_param("~date_format", "%Y-%m-%dT%H-%M-%S--%f")
        self.map_saver_wait_time = rospy.get_param("~map_saver_wait_time", 5.0)

        self.set_map_paths()

        if not os.path.isdir(self.map_dir):
            os.makedirs(self.map_dir)

        self.gmapping_launch_path = rospy.get_param("~gmapping_launch", self.default_launches_dir + "/gmapping.launch")
        self.amcl_launch_path = rospy.get_param("~amcl_launch", self.default_launches_dir + "/amcl.launch")
        self.map_saver_launch_path = rospy.get_param("~map_saver_launch", self.default_launches_dir + "/map_saver.launch")
        self.fake_map_launch_path = rospy.get_param("~fake_map_launch", self.default_launches_dir + "/fake_map.launch")

        self.gmapping_launcher = LaunchManager(self.gmapping_launch_path)
        self.amcl_launcher = LaunchManager(self.amcl_launch_path, map_path=self.map_path + ".yaml")
        self.map_saver_launcher = LaunchManager(self.map_saver_launch_path, map_path=self.map_path)
        self.fake_map_launcher = LaunchManager(self.fake_map_launch_path)

        self.launchers = [
            self.gmapping_launcher,
            self.amcl_launcher,
            self.map_saver_launcher,
            self.fake_map_launcher,
        ]

        self.switch_mode_srv = rospy.Service(self.service_ns_name + "/set_slam_mode", SetSlamMode, self.switch_mode_callback)
        self.get_mode_srv = rospy.Service(self.service_ns_name + "/get_slam_mode", GetSlamMode, self.get_mode_callback)

        # self.localize_rate = rostopic.ROSTopicHz(15)
        self.localize_topic = "/amcl_pose"
        # rospy.Subscriber(self.localize_topic, rospy.AnyMsg, self.localize_rate.callback_hz, callback_args=self.localize_topic, queue_size=1)

        # self.map_rate = rostopic.ROSTopicHz(15)
        self.map_topic = "/map"
        # rospy.Subscriber(self.map_topic, rospy.AnyMsg, self.map_rate.callback_hz, callback_args=self.map_topic, queue_size=1)

        # signal.signal(signal.SIGINT, lambda sig, frame: self.signal_handler(sig, frame))
    
    def get_publish_rate(self, rate_obj, topic):
        result = rate_obj.get_hz(topic)
        if result is None:
            return 0.0
        else:
            return result[0]

    def topic_exists(self, check_topic):
        for topic, msg_type in rospy.get_published_topics():
            if check_topic in topic:
                return True
        return False

    def is_publishing(self):
        if self.mode == self.LOCALIZE:
            # return self.get_publish_rate(self.localize_rate, self.localize_topic) > self.min_localize_rate_threshold
            return self.topic_exists(self.localize_topic)
        elif self.mode == self.MAPPING:
            # return self.get_publish_rate(self.map_rate, self.map_topic) > self.min_mapping_rate_threshold
            return self.topic_exists(self.map_topic)
        else:
            return True

    def set_map_paths(self):
        map_name = self.generate_map_name(self.map_name)
        self.map_path = os.path.join(self.map_dir, map_name)
        self.map_dir = os.path.dirname(self.map_path)
        self.map_name = os.path.basename(self.map_path)
    
    def switch_mode_callback(self, req):
        success, message = self.switch_mode(req.mode, req.map_name)
        return SetSlamModeResponse(success, message)
    
    def switch_mode(self, mode, map_name):
        rospy.loginfo("Requesting mode switch to '%s'" % mode)
        if not self.is_mode_valid(mode):
            return False, "Unknown mode '%s'. Valid modes: %s" % (mode, ", ".join(self.MODES))
        
        if self.mode == mode:
            return True, "Mode is already set to %s" % mode

        if mode == self.IDLE:
            self.fake_map_launcher.start()
        else:
            self.fake_map_launcher.stop()
        
        if self.mode == self.MAPPING:  # if mode was mapping, save the map
            self.save_map()

        if map_name:
            self.map_name = map_name
            self.set_map_paths()
            rospy.loginfo("Setting map path to %s" % self.map_path)
            self.amcl_launcher.set_args(map_path=self.map_path + ".yaml")
            self.map_saver_launcher.set_args(map_path=self.map_path)

        if mode == self.LOCALIZE:
            self.gmapping_launcher.stop()
            self.amcl_launcher.start()
        elif mode == self.MAPPING:
            self.gmapping_launcher.start()
            self.amcl_launcher.stop()
        else:
            self.gmapping_launcher.stop()
            self.amcl_launcher.stop()
        
        self.mode = mode
        rospy.loginfo("Mode switched to %s" % self.mode)
        return True, "Mode is now set to %s" % self.mode
    
    def get_mode_callback(self, req):
        return GetSlamModeResponse(self.mode, self.is_publishing())

    def generate_map_name(self, name_format):
        date_str = datetime.now().strftime(self.date_format)
        name = name_format.format_map(defaultdict(str, date=date_str))
        return name
    
    def is_mode_valid(self, mode):
        return mode in self.MODES

    def run(self):
        self.switch_mode(self.start_mode, self.map_name)
        
        rospy.spin()
        
    def stop_all(self):
        for launcher in self.launchers:
            launcher.stop()

    def shutdown_hook(self):
        try:
            rospy.loginfo("shutdown called")
            if self.mode == self.MAPPING:
                self.save_map()
        finally:
            self.stop_all()
    
    def save_map(self):
        rospy.loginfo("Saving map to %s. Waiting %0.1f seconds for map saver to finish." % (
            self.map_path, self.map_saver_wait_time)
        )
        self.map_saver_launcher.start()
        self.map_saver_launcher.join(timeout=self.map_saver_wait_time)
        rospy.loginfo("Map saved!")

    # def signal_handler(self, sig, frame):
    #     self.shutdown_hook()
    #     self.stop_event.set()
    #     sys.exit(0)


if __name__ == "__main__":
    node = DodobotLaserSlam()
    try:
        node.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        rospy.loginfo("Exiting %s node" % node.node_name)
