#!/usr/bin/env python3
import os
import sys
import signal
import rospy
import rospkg
import threading
from datetime import datetime
from subprocess import Popen, PIPE
from collections import defaultdict


class LaunchManager:
    roslaunch_exec = "roslaunch"
    bash_env_exec = "/usr/bin/env"
    def __init__(self, launch_path, *args, **kwargs):
        
        args_list = []
        for arg in args:
            args_list.append(str(arg))
        for name, value in kwargs.items():
            args_list.append("%s:=%s" % (name, value))
        
        if len(args_list) > 0:
            self.roslaunch_args = [self.bash_env_exec, self.roslaunch_exec, launch_path] + args_list
        else:
            self.roslaunch_args = [self.bash_env_exec, self.roslaunch_exec, launch_path]
        
        self.process = None
    
    def start(self):
        rospy.loginfo("roslaunch args: %s" % self.roslaunch_args)
        self.process = Popen(self.roslaunch_args, preexec_fn=os.setpgrp)  #, stdout=PIPE, stderr=PIPE)
    
    def stop(self):
        if self.is_running():
            self.process.send_signal(signal.SIGINT)
    
    def join(self, timeout):
        self.process.wait()
    
    def is_running(self):
        return self.process is not None and self.process.poll() is None


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
        self.map_name = rospy.get_param("~map_name", "map-{date}")
        self.date_format = rospy.get_param("~date_format", "%Y-%m-%dT%H-%M-%S--%f")

        map_name = self.generate_map_name(self.map_name)
        self.map_path = os.path.join(self.map_dir, map_name)
        self.map_dir = os.path.dirname(self.map_path)
        self.map_name = os.path.basename(self.map_path)

        if not os.path.isdir(self.map_dir):
            os.makedirs(self.map_dir)

        self.gmapping_launch_path = rospy.get_param("~gmapping_launch", self.default_launches_dir + "/gmapping.launch")
        self.amcl_launch_path = rospy.get_param("~amcl_launch", self.default_launches_dir + "/amcl.launch")
        self.map_saver_launch_path = rospy.get_param("~map_saver_launch", self.default_launches_dir + "/map_saver.launch")

        self.gmapping_launcher = LaunchManager(self.gmapping_launch_path)
        self.amcl_launcher = LaunchManager(self.amcl_launch_path, map_path=self.map_path + ".yaml")
        self.map_saver_launcher = LaunchManager(self.map_saver_launch_path, map_path=self.map_path)

        self.launchers = [
            self.gmapping_launcher,
            self.amcl_launcher,
            self.map_saver_launcher,
        ]

        # signal.signal(signal.SIGINT, lambda sig, frame: self.signal_handler(sig, frame))
    
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
        rospy.loginfo("shutdown called")
        if self.mode == "mapping":
            rospy.loginfo("Saving map to %s. Waiting 5 seconds." % self.map_path)
            self.map_saver_launcher.start()
            self.map_saver_launcher.join(timeout=5.0)
            rospy.loginfo("Map saved!")
        
        self.stop_all()
    
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
