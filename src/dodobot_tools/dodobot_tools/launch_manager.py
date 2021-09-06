import os
import time
import rospy
import psutil
import signal
from subprocess import Popen, PIPE, TimeoutExpired


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
        if self.is_running():
            rospy.loginfo("roslaunch already running. args: %s" % self.roslaunch_args)
            return False
        else:
            rospy.loginfo("roslaunch args: %s" % self.roslaunch_args)
            self.process = Popen(self.roslaunch_args, preexec_fn=os.setpgrp)  #, stdout=PIPE, stderr=PIPE)
            return True
    
    def stop(self):
        if self.process is not None:
            rospy.loginfo("Sending SIGINT to %s" % self.roslaunch_args)
            self.process.send_signal(signal.SIGINT)
            return True
        else:
            return False
    
    def join(self, timeout):
        try:
            if self.process is not None:
                self.process.wait(timeout=timeout)
            return True
        except TimeoutExpired:
            return False
    
    def is_running(self):
        return self.process is not None and self.process.poll() is None
