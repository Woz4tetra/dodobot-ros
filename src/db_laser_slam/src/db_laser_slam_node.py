#!/usr/bin/env python3
import os
import rospy
import rospkg
import roslaunch
import threading
from datetime import datetime
from collections import defaultdict


class ProcessListener(roslaunch.pmon.ProcessListener):
    def __init__(self):
        self._is_running = False
        self.stop_event = threading.Event()
    
    def start(self):
        self._is_running = True
    
    def is_running(self):
        return self._is_running

    def process_died(self, name, exit_code):
        rospy.logwarn("%s died with code %s", name, exit_code)
        self._is_running = False
        self.stop_event.set()
    
    def wait(self, timeout=None):
        self.stop_event.wait(timeout=timeout)


class LaunchManager:
    def __init__(self, launch_path, *args, **kwargs):
        self.uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(self.uuid)
        self.pmon = ProcessListener()

        args_list = []
        for arg in args:
            args_list.append(str(arg))
        for name, value in kwargs.items():
            args_list.append("%s:=%s" % (name, value))

        roslaunch_args = [launch_path] + args_list
        roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(args_list)[0], roslaunch_args)]
        self.launch = roslaunch.parent.ROSLaunchParent(
            self.uuid,
            [roslaunch_file],
            process_listeners=[self.pmon],
        )
    
    def start(self):
        self.pmon.start()
        self.launch.start()
    
    def stop(self):
        self.launch.shutdown()
    
    def join(self, timeout):
        self.pmon.wait(timeout)
    
    def is_running(self):
        return self.pmon.is_running()


class DodobotLaserSlam:
    def __init__(self):
        self.node_name = "db_laser_slam"
        rospy.init_node(
            self.node_name
            # disable_signals=True
            # log_level=rospy.DEBUG
        )
        rospy.on_shutdown(self.shutdown_hook)

        self.rospack = rospkg.RosPack()
        self.package_dir = self.rospack.get_path(self.node_name)
        self.default_maps_dir = self.package_dir + "/maps"
        self.default_launches_dir = self.package_dir + "/launch/sublaunch"

        self.mode = rospy.get_param("~mode", "mapping")
        self.map_dir = rospy.get_param("~map_dir", self.default_maps_dir)
        self.map_name = rospy.get_param("~map_name", "map-{date}.yaml")
        self.date_format = rospy.get_param("~date_format", "%Y-%m-%dT%H-%M-%S--%f")
        
        map_name = self.generate_map_name(self.map_name)
        self.map_path = os.path.join(self.map_dir, map_name)

        self.gmapping_launch_path = rospy.get_param("~gmapping_launch", self.default_launches_dir + "/gmapping.launch")
        self.amcl_launch_path = rospy.get_param("~amcl_launch", self.default_launches_dir + "/amcl.launch")
        self.map_saver_launch_path = rospy.get_param("~map_saver_launch", self.default_launches_dir + "/map_saver.launch")

        self.gmapping_launcher = LaunchManager(self.gmapping_launch_path)
        self.amcl_launcher = LaunchManager(self.amcl_launch_path, map_path=self.map_path)
        self.map_saver_launcher = LaunchManager(self.map_saver_launch_path, "-f %s" % self.map_path)
    
    def generate_map_name(self, name_format):
        date_str = datetime.now().strftime(self.date_format)
        name = name_format.format_map(defaultdict(str, date=date_str))
        return name
    
    def run(self):
        if self.mode == "mapping":
            self.gmapping_launcher.start()
        elif self.mode == "localize":
            self.amcl_launcher.start()
        rospy.spin()

    def stop_all(self):
        self.gmapping_launcher.stop()
        self.amcl_launcher.stop()
        self.map_saver_launcher.stop()

    def shutdown_hook(self):
        if self.mode == "mapping":
            rospy.loginfo("Saving map to %s" % self.map_path)
            self.map_saver_launcher.start()
            self.map_saver_launcher.join(timeout=30.0)
        
        self.stop_all()


if __name__ == "__main__":
    node = DodobotLaserSlam()
    try:
        node.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        rospy.loginfo("Exiting %s node" % node.node_name)
