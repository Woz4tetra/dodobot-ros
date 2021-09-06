#!/usr/bin/env python3
import rospy
import rospkg

from std_srvs.srv import Trigger, TriggerResponse

from dodobot_tools.launch_manager import LaunchManager


class DodobotCameraLauncher:
    def __init__(self):
        self.node_name = "db_launcher"
        rospy.init_node(
            self.node_name
            # disable_signals=True
            # log_level=rospy.DEBUG
        )
        rospy.on_shutdown(self.shutdown_hook)

        self.rospack = rospkg.RosPack()
        self.package_dir = self.rospack.get_path(self.package_name)
        self.default_launches_dir = self.package_dir + "/launch"

        self.camera_launch_path = rospy.get_param("~camera_launch", self.default_launches_dir + "/db_camera.launch")
        self.record_launch_path = rospy.get_param("~record_launch", self.default_launches_dir + "/record_camera.launch")

        self.camera_launcher = LaunchManager(self.camera_launch_path)
        self.record_launcher = LaunchManager(self.record_launch_path)

        self.launchers = [
            self.camera_launcher,
            self.record_launcher,
        ]

        self.start_camera_srv = rospy.Service(self.service_ns_name + "/start_camera", Trigger, self.start_camera_callback)
        self.stop_camera_srv = rospy.Service(self.service_ns_name + "/stop_camera", Trigger, self.stop_camera_callback)
        self.start_record_srv = rospy.Service(self.service_ns_name + "/start_record", Trigger, self.start_record_callback)
        self.stop_record_srv = rospy.Service(self.service_ns_name + "/stop_record", Trigger, self.stop_record_callback)

        rospy.loginfo("%s init complete" % self.node_name)

    def start_camera_callback(self, req):
        started = self.camera_launcher.start()
        return TriggerResponse(started, "Camera started" if started else "Camera is already running!")
    
    def stop_camera_callback(self, req):
        stopped = self.camera_launcher.stop()
        self.record_launcher.stop()
        return TriggerResponse(stopped, "Camera stopped" if stopped else "Camera is already stopped!")
    
    def start_record_callback(self, req):
        started = self.record_launcher.start()
        return TriggerResponse(started, "Recording started" if started else "Recording is already running!")
    
    def stop_record_callback(self, req):
        stopped = self.record_launcher.stop()
        return TriggerResponse(stopped, "Recording stopped" if stopped else "Recording is already stopped!")
    
    def run(self):
        rospy.spin()
        
    def stop_all(self):
        for launcher in self.launchers:
            launcher.stop()

    def shutdown_hook(self):
        self.stop_all()


if __name__ == "__main__":
    node = DodobotCameraLauncher()
    try:
        node.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        rospy.loginfo("Exiting %s node" % node.node_name)
