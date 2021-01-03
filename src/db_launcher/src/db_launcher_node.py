#!/usr/bin/env python
import rospy
import Queue
import roslaunch
from std_msgs.msg import String

from db_parsing.msg import DodobotFunctionsListing
from db_parsing.msg import DodobotFunctions
from db_parsing.msg import DodobotState
from db_parsing.msg import DodobotNotify


class FunctionGroup(object):
    def __init__(self):
        self.functions = []
        self.fn_mapping = {}

    @classmethod
    def from_dict(cls, kwargs):
        self = cls()
        for serial, callback in kwargs.items():
            self.add_fn(serial, callback)
        return self

    def add_fn(self, serial, callback):
        if serial not in self.functions:
            self.functions.append(serial)
        self.fn_mapping[serial] = callback

    def remove_fn(self, serial):
        self.functions.remove(serial)
        self.fn_mapping.pop(serial)

    def select(self, serial):
        if serial in self.fn_mapping:
            return self.fn_mapping[serial]
        else:
            return None

class FunctionsList(object):
    def __init__(self):
        self.group_names = []
        self.groups = {}

    @classmethod
    def from_dict(cls, kwargs):
        self = cls()
        self.update_from_dict(kwargs)
        return self

    def update_from_dict(self, kwargs):
        for serial, group_args in kwargs.items():
            self.add_group(serial, FunctionGroup.from_dict(group_args))

    @classmethod
    def from_list(cls, args):
        self = cls()
        for serial in args:
            self.add_group(serial, None)
        return self

    def add_group(self, serial, group):
        if serial not in self.group_names:
            self.group_names.append(serial)
        self.groups[serial] = group

    def select(self, serial):
        for group in self.groups.values():
            fn = group.select(serial)
            if fn:
                fn()

    def to_dodobot_listing(self):
        listing = DodobotFunctionsListing()
        for serial in self.group_names:
            group = self.groups[serial]
            fns = DodobotFunctions()
            for serial in group.functions:
                fns.functions.append(serial)

            listing.menu.append(fns)
        return listing

class ProcessListener(roslaunch.pmon.ProcessListener):
    def __init__(self, manager):
        self.is_running = False
        self.manager = manager

    def process_died(self, name, exit_code):
        rospy.logwarn("%s died with code %s", name, exit_code)


class LaunchManager(object):
    def __init__(self):
        self.uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(self.uuid)
        self.launches = {}
        self.launch_args = {}
        self.processes = {}
        self.requests = Queue.Queue()

    def add_launch(self, local_name, package, filename, **kwargs):
        if local_name in self.launches:
            rospy.logwarn("%s is already added" % local_name)
            return

        cli_kwargs = ["%s:=%s" % (key, value) for key, value in kwargs.items()]
        cli_args = [package, filename] + cli_kwargs
        proc_listener = ProcessListener(self)

        self.launch_args[local_name] = cli_args
        self.processes[local_name] = proc_listener

    def start_launch(self, local_name):
        self.requests.put((local_name, "start"))

    def _start_launch_from_req(self, local_name):
        if self.processes[local_name].is_running:
            rospy.logwarn("%s is already started" % local_name)
            return
        self.processes[local_name].is_running = True

        cli_args = self.launch_args[local_name]
        proc_listener = self.processes[local_name]

        roslaunch_file = roslaunch.rlutil.resolve_launch_arguments(cli_args)
        parent = roslaunch.parent.ROSLaunchParent(self.uuid, roslaunch_file, process_listeners=[proc_listener])
        self.launches[local_name] = parent
        parent.start()

    def is_running(self, local_name):
        return self.processes[local_name].is_running

    def spin_once(self):
        self._process_requests()
        for launch in self.launches.values():
            launch.spin_once()

    def stop_launch(self, local_name):
        self.requests.put((local_name, "stop"))

    def _stop_launch_from_req(self, local_name):
        if not self.processes[local_name].is_running:
            rospy.logwarn("%s is already stopped" % local_name)
            return
        self.processes[local_name].is_running = False
        self.launches[local_name].shutdown()

    def _process_requests(self):
        while not self.requests.empty():
            local_name, command = self.requests.get()
            if command == "start":
                self._start_launch_from_req(local_name)
            elif command == "stop":
                self._stop_launch_from_req(local_name)

    def stop_all(self):
        for name in self.launches:
            self.stop_launch(name)


class DodobotLauncher(object):
    def __init__(self):
        self.node_name = "db_launcher"
        rospy.init_node(
            self.node_name
            # disable_signals=True
            # log_level=rospy.DEBUG
        )
        rospy.on_shutdown(self.shutdown_hook)

        self.launcher = LaunchManager()

        self.camera_launch = "camera"
        self.launcher.add_launch(self.camera_launch, self.node_name, "camera.launch")

        self.function_list_pub = rospy.Publisher("functions", DodobotFunctionsListing, queue_size=50)
        self.notif_pub = rospy.Publisher("notify", DodobotNotify, queue_size=10)
        self.selected_fn_sub = rospy.Subscriber("selected_fn", DodobotFunctionsListing, self.selected_fn_callback, queue_size=10)
        self.state_sub = rospy.Subscriber("state", DodobotState, self.robot_state_callback, queue_size=10)

        self.prev_ready_state = False

        self.groups = FunctionsList.from_list(["hardware", "rtabmap", "navigation", "objects"])
        self.groups.update_from_dict({
            "hardware": {
                "Start/stop camera": self.camera_callback,
            },
            "rtabmap": {
                "Start new map": self.callback,
                "Load last map": self.callback,
                "Stop mapping": self.callback,
            },
            "navigation": {
                "Name this location": self.callback,
                "Go to ...": self.callback,
                "Explore": self.callback,
            },
            "objects": {
                "Pickup ...": self.callback,
                "Deliver ...": self.callback,
            }
        })
        rospy.loginfo("%s node started" % self.node_name)


    def camera_callback(self):
        rospy.loginfo("Camera callback")
        if self.launcher.is_running(self.camera_launch):
            rospy.loginfo("Stopping camera")
            self.launcher.stop_launch(self.camera_launch)
            self.send_notification(0, "Stop camera", 1000)
        else:
            rospy.loginfo("Starting camera")
            self.launcher.start_launch(self.camera_launch)
            self.send_notification(0, "Start camera", 1000)

    def send_notification(self, level, message, timeout):
        msg = DodobotNotify()
        msg.level = level
        msg.message = message
        msg.timeout = timeout
        self.notif_pub.publish(msg)

    def callback(self):
        print "callback"
        self.send_notification(0, "Ack", 1000)

    def selected_fn_callback(self, msg):
        self.groups.select(msg.selected)

    def robot_state_callback(self, msg):
        if msg.is_ready and msg.is_ready != self.prev_ready_state:
            self.function_list_pub.publish(self.groups.to_dodobot_listing())
            self.prev_ready_state = msg.is_ready

    def run(self):
        # rospy.spin()
        rate = rospy.Rate(10.0)
        while True:
            self.launcher.spin_once()
            rate.sleep()
            if rospy.is_shutdown():
                return

    def shutdown_hook(self):
        self.launcher.stop_all()


if __name__ == "__main__":
    node = DodobotLauncher()
    try:
        node.run()

    except rospy.ROSInterruptException:
        pass

    finally:
        rospy.loginfo("Exiting %s node" % node.node_name)
