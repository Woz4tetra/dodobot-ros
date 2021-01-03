#!/usr/bin/env python
import rospy
# import roslaunch
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

class DodobotLauncher(object):
    def __init__(self):
        self.node_name = "db_launcher"
        rospy.init_node(
            self.node_name
            # disable_signals=True
            # log_level=rospy.DEBUG
        )
        rospy.on_shutdown(self.shutdown_hook)

        self.uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(self.uuid)
        self.launches = []

        self.function_list_pub = rospy.Publisher("functions", DodobotFunctionsListing, queue_size=50)
        self.notif_pub = rospy.Publisher("notify", DodobotNotify, queue_size=10)
        self.selected_fn_sub = rospy.Subscriber("selected_fn", DodobotFunctionsListing, self.selected_fn_callback, queue_size=10)
        self.state_sub = rospy.Subscriber("state", DodobotState, self.robot_state_callback, queue_size=10)

        self.prev_ready_state = False

        self.groups = FunctionsList.from_list(["hardware", "rtabmap", "navigation", "objects"])
        self.groups.update_from_dict({
            "hardware": {
                "Start/stop camera": self.callback,
            },
            "rtabmap": {
                "Start new map": self.callback,
                "Load last map": self.callback,
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

        self.camera_launch = "camera"

    def camera_callback(self):
        if self.camera_launch not in self.launches:
            self.add_launch(self.camera_launch, self.node_name, "camera.launch")

        self.camera_launch.start()


    def add_launch(self, local_name, package, filename, *args):
        if local_name in self.launches:
            rospy.logwarn("%s is already launched" % local_name)
            return
        cli_args = [package, filename] + list(args)
        roslaunch_file = roslaunch.rlutil.resolve_launch_arguments(cli_args)
        roslaunch_args = cli_args[2:]
        launch = roslaunch.parent.ROSLaunchParent(self.uuid, [roslaunch_file, roslaunch_args])
        self.launches[local_name] = launch

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
        rospy.spin()
        # rate = rospy.Rate(0.5)
        # while True:
        #     if rospy.is_shutdown():
        #         return
        #     rate.sleep()

    def shutdown_hook(self):
        for launch in self.launches.values():
            launch.shutdown()


if __name__ == "__main__":
    node = DodobotLauncher()
    try:
        node.run()

    except rospy.ROSInterruptException:
        pass

    finally:
        rospy.loginfo("Exiting %s node" % node.node_name)
