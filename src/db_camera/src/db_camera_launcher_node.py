#!/usr/bin/env python3
import rospy
import rospkg
import rostopic

from std_srvs.srv import Trigger, TriggerResponse

from sensor_msgs.msg import Image

from dodobot_tools.launch_manager import LaunchManager


class DodobotCameraLauncher:
    def __init__(self):
        self.package_name = "db_camera"
        self.node_name = "db_camera_launcher"
        rospy.init_node(
            self.node_name
            # disable_signals=True
            # log_level=rospy.DEBUG
        )
        rospy.on_shutdown(self.shutdown_hook)

        self.rospack = rospkg.RosPack()
        self.package_dir = self.rospack.get_path(self.package_name)
        self.default_launches_dir = self.package_dir + "/launch"

        self.service_ns_name = rospy.get_param("~service_ns_name", "/dodobot")
        self.camera_launch_path = rospy.get_param("~camera_launch", self.default_launches_dir + "/db_camera.launch")
        self.record_launch_path = rospy.get_param("~record_launch", self.default_launches_dir + "/record_camera.launch")
        self.ring_buffer_length = rospy.get_param("~ring_buffer_length", 25)
        self.min_rate_threshold = rospy.get_param("~min_rate_threshold", 5.0)

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
        self.is_camera_running_srv = rospy.Service(self.service_ns_name + "/is_camera_running", Trigger, self.is_camera_running_callback)

        self.camera_rate = rostopic.ROSTopicHz(-1)
        self.camera_topic = "/camera/color/image_raw"
        rospy.Subscriber(self.camera_topic, rospy.AnyMsg, self.camera_rate.callback_hz, callback_args=self.camera_topic, queue_size=1)

        rospy.loginfo("%s init complete" % self.node_name)

    def get_publish_rate(self):
        rospy.sleep(1.0)
        result = self.camera_rate.get_hz(self.camera_topic)
        if result is None:
            return 0.0
        else:
            return result[0]

    def start_camera_callback(self, req):
        started = self.camera_launcher.start()
        return TriggerResponse(started, "Camera started" if started else "Camera is already running!")
    
    def is_camera_running_callback(self, req):
        if self.camera_launcher.is_running():
            rate = self.get_publish_rate()
            if rate > self.min_rate_threshold:
                return TriggerResponse(True, "Camera is running and publishing at %0.2f Hz" % rate)
            else:
                return TriggerResponse(False, "Camera is running but not publishing above the threshold: %0.2f" % rate)
        else:
            return TriggerResponse(False, "Camera is not started")

    def stop_camera_callback(self, req):
        stopped = self.camera_launcher.stop()
        self.record_launcher.stop()
        return TriggerResponse(stopped, "Camera stopped" if stopped else "Camera is already stopped!")
    
    def start_record_callback(self, req):
        started = self.record_launcher.start()
        return TriggerResponse(started, "Recording started" if started else "Recording is already running!")
    
    def is_camera_recording_callback(self, req):
        TriggerResponse()

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
