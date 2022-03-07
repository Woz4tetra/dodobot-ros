import os
import rospy
import rostopic
import signal
from subprocess import Popen, TimeoutExpired


class TopicListener:
    def __init__(self, topic: str, min_rate: float, queue_size=15):
        self.topic = topic
        self.min_rate = min_rate
        self.rate = rostopic.ROSTopicHz(queue_size)
        self.subscriber = rospy.Subscriber(self.topic, rospy.AnyMsg, self.rate.callback_hz, callback_args=self.topic)
    
    def topic_exists(self):
        for topic, msg_type in rospy.get_published_topics():
            if self.topic in topic:
                return True
        return False

    def is_active(self):
        if not self.topic_exists():
            return False
        if self.min_rate is None:
            return True
        else:
            return self.get_rate() > self.min_rate

    def get_rate(self, delay=1.0):
        if delay > 0.0:
            rospy.sleep(delay)
        result = self.rate.get_hz(self.topic)
        if result is None:
            return 0.0
        else:
            return result[0]


class LaunchManager:
    roslaunch_exec = "roslaunch"
    bash_env_exec = "/usr/bin/env"
    def __init__(self, launch_path, *args, **kwargs):
        self.launch_path = launch_path
        self.set_args(*args, **kwargs)
        self.process = None
    
    def set_args(self, *args, **kwargs):
        args_list = []
        for arg in args:
            args_list.append(str(arg))
        for name, value in kwargs.items():
            args_list.append("%s:=%s" % (name, value))
        
        if len(args_list) > 0:
            self.roslaunch_args = [self.bash_env_exec, self.roslaunch_exec, self.launch_path] + args_list
        else:
            self.roslaunch_args = [self.bash_env_exec, self.roslaunch_exec, self.launch_path]
        

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
