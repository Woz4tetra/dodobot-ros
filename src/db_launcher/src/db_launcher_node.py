#!/usr/bin/env python3
import os
import rospy
import rospkg

from db_launcher.srv import SetLaunch, SetLaunchResponse
from db_launcher.srv import GetLaunch, GetLaunchResponse

from db_tools.launch_manager import LaunchManager
from db_tools.launch_manager import TopicListener


class DodobotLauncher:
    def __init__(self):
        self.node_name = "db_launcher"
        rospy.init_node(
            self.node_name
            # disable_signals=True
            # log_level=rospy.DEBUG
        )
        rospy.on_shutdown(self.shutdown_hook)

        self.launch_config = rospy.get_param("~launches", None)
        self.service_ns = rospy.get_param("~service_ns", "/dodobot")
        self.ring_buffer_length = rospy.get_param("~ring_buffer_length", 10)
        assert self.launch_config is not None

        self.rospack = rospkg.RosPack()
        self.launchers = self.build_launchers(self.launch_config)
        self.listeners = self.build_topic_listeners(self.launch_config)
        self.set_launch_srv = rospy.Service(self.service_ns + "/set_launch", SetLaunch, self.set_launch_callback)
        self.get_launch_srv = rospy.Service(self.service_ns + "/get_launch", GetLaunch, self.get_launch_callback)

        rospy.loginfo("%s init complete" % self.node_name)

    def build_launchers(self, launch_config):
        launchers = {}
        for name, data in launch_config.items():
            package = data.get("package", "")
            path = data.get("path", "")
            on_start = data.get("on_start", False)

            if not package:
                if not os.path.isfile(path):
                    rospy.logerr("Package is not provided for %s and path is not valid: %s. Skipping launch" % (name, path))
                    continue
                else:
                    launch_path = path
            else:
                package_dir = self.rospack.get_path(package)
                launch_dir = package_dir + "/launch"

                launch_path = launch_dir + "/" + path
            
            launcher = LaunchManager(launch_path)
            launchers[name] = launcher
            if on_start:
                launcher.start()
        return launchers

    def build_topic_listeners(self, launch_config):
        listeners = {}
        for name, data in launch_config.items():
            launch_listeners = {}
            topics = data.get("topics", {})
            for topic, min_rate in topics.items():
                listener = TopicListener(topic, min_rate)
                launch_listeners[topic] = listener
            listeners[name] = launch_listeners
        return listeners

    def set_launch_callback(self, req):
        if req.name not in self.launchers:
            return SetLaunchResponse(False, "Not a valid launch name: '%s'" % (req.name))

        if req.mode == 1:  # SetLaunch.MODE_START
            self.launchers[req.name].start()
        elif req.mode == 2:  # SetLaunch.MODE_STOP
            self.launchers[req.name].stop()
        else:
            return SetLaunchResponse(False, "Invalid mode '%s' for launch '%s'" % (req.mode, req.name))
        return SetLaunchResponse(True, ("Starting" if req.mode == 1 else "Stopping") + " " + str(req.name))
    
    def get_launch_callback(self, req):
        if req.name not in self.launchers:
            return GetLaunchResponse(False, "Not a valid launch name: '%s'" % (req.name))

        if self.launchers[req.name].is_running():
            if req.name in self.listeners:
                for topic, listener in self.listeners[req.name].items():
                    if listener.is_active():
                        return GetLaunchResponse(False,
                            "Launch %s is running, but topic '%s' not publishing above %s Hz. Publishing at %s Hz" % (
                                req.name, topic, listener.min_rate, listener.get_rate()
                            ))
            return GetLaunchResponse(True, "Launch %s is running")
        else:
            return GetLaunchResponse(False, "Launch %s is not running" % req.name)

    def run(self):
        rospy.spin()
    
    def stop_all(self):
        for launcher in self.launchers.values():
            launcher.stop()

    def shutdown_hook(self):
        self.stop_all()
    


if __name__ == "__main__":
    node = DodobotLauncher()
    try:
        node.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        rospy.loginfo("Exiting %s node" % node.node_name)
