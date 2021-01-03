#!/usr/bin/env python
import os
import Queue
from datetime import datetime

import rospy
import roslaunch

from std_msgs.msg import String

from db_parsing.msg import DodobotFunctionsListing
from db_parsing.msg import DodobotFunctions
from db_parsing.msg import DodobotState
from db_parsing.msg import DodobotNotify

from db_parsing.srv import DodobotSetState, DodobotSetStateResponse

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
        # self.is_running = False
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

        cli_args = [package, filename, kwargs]
        proc_listener = ProcessListener(self)

        self.launch_args[local_name] = cli_args
        self.processes[local_name] = proc_listener

    def update_kwargs(self, local_name, **kwargs):
        self.launch_args[local_name][2].update(**kwargs)

    def start_launch(self, local_name):
        self.requests.put((local_name, "start"))

    def _start_launch_from_req(self, local_name):
        if self.processes[local_name].is_running:
            rospy.logwarn("%s is already started" % local_name)
            return
        self.processes[local_name].is_running = True

        package, filename, kwargs = self.launch_args[local_name]
        cli_kwargs = ["%s:=%s" % (key, value) for key, value in kwargs.items()]
        cli_args = [package, filename] + cli_kwargs
        rospy.loginfo("Launching with args: %s" % str(cli_args))
        proc_listener = self.processes[local_name]

        roslaunch_file = roslaunch.rlutil.resolve_launch_arguments(cli_args)
        launch_cfg = [(roslaunch_file[0], cli_kwargs)]

        parent = roslaunch.parent.ROSLaunchParent(self.uuid, launch_cfg, process_listeners=[proc_listener])
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
        self.rtabmap_new_launch = "rtabmap_new"
        self.rtabmap_load_launch = "rtabmap_load"
        self.move_base_launch = "move_base"
        self.launcher.add_launch(self.camera_launch, self.node_name, "camera.launch")
        self.launcher.add_launch(self.rtabmap_new_launch, self.node_name, "rtabmap.launch", localization=False)
        self.launcher.add_launch(self.rtabmap_load_launch, self.node_name, "rtabmap.launch", localization=True)
        self.launcher.add_launch(self.move_base_launch, self.node_name, "move_base.launch")

        self.rtabmap_dir = rospy.get_param("~rtabmap_dir", "/home/ben/rtabmaps")
        self.rtabmap_filename = rospy.get_param("~rtabmap_filename", "rtabmap_%Y-%m-%d--%H-%M-%S,%f.db")
        if not os.path.isdir(self.rtabmap_dir):
            os.makedirs(self.rtabmap_dir)

        self.function_list_pub = rospy.Publisher("functions", DodobotFunctionsListing, queue_size=50)
        self.notif_pub = rospy.Publisher("notify", DodobotNotify, queue_size=10)
        self.selected_fn_sub = rospy.Subscriber("selected_fn", DodobotFunctionsListing, self.selected_fn_callback, queue_size=10)
        self.state_sub = rospy.Subscriber("state", DodobotState, self.robot_state_callback, queue_size=10)

        self.set_robot_state_service_name = "set_state"
        rospy.loginfo("Waiting for service %s" % self.set_robot_state_service_name)
        self.set_robot_state = rospy.ServiceProxy(self.set_robot_state_service_name, DodobotSetState)
        rospy.loginfo("%s service is ready" % self.set_robot_state_service_name)

        self.prev_ready_state = False

        self.groups = FunctionsList.from_list(["hardware", "rtabmap", "navigation", "objects"])
        self.groups.update_from_dict({
            "hardware": {
                "Start/stop camera": self.camera_callback,
            },
            "rtabmap": {
                "Start new map": self.rtabmap_new_map,
                "Load last map": self.rtabmap_recent_map,
                "Stop mapping": self.rtabmap_stop,
            },
            "navigation": {
                "Name this location": self.name_location_callback,
                # "Go to ...": self.callback,
                "Explore": self.callback,
                "Stop move_base": self.stop_move_base,
            },
            "objects": {
                "Pickup ...": self.callback,
                "Deliver ...": self.callback,
            }
        })
        rospy.loginfo("%s node started" % self.node_name)

    # ---
    # Camera callbacks
    # ---

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

    def rtabmap_new_map(self):
        if self.launcher.is_running(self.rtabmap_new_launch):
            rospy.loginfo("rtabmap mapping already running")
            return

        if not self.launcher.is_running(self.camera_launch):
            self.launcher.start_launch(self.camera_launch)
        self.set_robot_state(True, True)

        if self.launcher.is_running(self.rtabmap_load_launch):
            self.launcher.stop_launch(self.rtabmap_load_launch)

        self.launcher.update_kwargs(self.rtabmap_new_launch, map_name=self.get_map_name())
        self.launcher.start_launch(self.rtabmap_new_launch)
        self.send_notification(0, "New map", 1000)

    # ---
    # RTAB-Map callbacks
    # ---
    def rtabmap_recent_map(self):
        if self.launcher.is_running(self.rtabmap_load_launch):
            rospy.loginfo("rtabmap recent already running")
            return

        map_name = self.get_recent_rtabmap()
        if map_name is None:
            self.rtabmap_new_map()
            return

        if not self.launcher.is_running(self.camera_launch):
            self.launcher.start_launch(self.camera_launch)
        self.set_robot_state(True, True)

        if self.launcher.is_running(self.rtabmap_new_launch):
            self.launcher.stop_launch(self.rtabmap_new_launch)

        self.launcher.update_kwargs(self.rtabmap_load_launch, map_name=map_name)
        self.launcher.start_launch(self.rtabmap_load_launch)
        self.send_notification(0, "Recent map", 1000)

    def rtabmap_stop(self):
        if self.launcher.is_running(self.rtabmap_load_launch):
            self.launcher.stop_launch(self.rtabmap_load_launch)
        if self.launcher.is_running(self.rtabmap_new_launch):
            self.launcher.stop_launch(self.rtabmap_new_launch)
        self.send_notification(0, "Stop map", 1000)

    def get_map_name(self):
        now = datetime.now()
        filename = now.strftime(self.rtabmap_filename)
        path = os.path.join(self.rtabmap_dir, filename)
        return path

    def get_recent_rtabmap(self):
        recent_filename = None
        now = datetime.now()
        recent_date = None
        for filename in os.listdir(self.rtabmap_dir):
            try:
                date = datetime.strptime(filename, self.rtabmap_filename)
            except ValueError as e:
                rospy.logwarn(e)
            dt = now - date
            if recent_date is None or recent_date - date < dt:
                recent_date = date
                recent_filename = filename

        if recent_filename is None:
            return None
        else:
            return os.path.join(self.rtabmap_dir, recent_filename)

    # ---
    # Navigation callbacks
    # ---

    def name_location_callback(self):
        if not self.launcher.is_running(self.rtabmap_load_launch) and \
                not self.launcher.is_running(self.rtabmap_new_launch):
            self.send_notification(1, "Map stopped", 2000)
            return

    def go_to_location(self, name):
        pass

    def stop_move_base(self):
        if self.launcher.is_running(self.move_base_launch):
            self.launcher.stop_launch(self.move_base_launch)

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
