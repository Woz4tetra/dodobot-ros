#!/usr/bin/env python3
import os
import cv2
import json
import datetime

import rospy

from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo

from db_tools.rosbag_to_file import utils
from db_tools.rosbag_to_file.rosbag_to_json import StreamArray


class RecordToVideoFile:
    def __init__(self):
        self.name = "record_to_video_file"
        rospy.init_node(
            self.name
            # disable_signals=True
            # log_level=rospy.DEBUG
        )
        rospy.on_shutdown(self.shutdown_hook)
        self.video_path = rospy.get_param("~video_path", "./video.mp4")
        self.video_fps = rospy.get_param("~video_fps", 30)
        self.rotate = rospy.get_param("~rotate", 0)
        self.bridge = CvBridge()
        self.info_sub = rospy.Subscriber("camera_info", CameraInfo, self.info_callback, queue_size=10)
        self.image_sub = rospy.Subscriber("image", Image, self.image_callback, queue_size=10)

        self.video_path = datetime.datetime.now().strftime(self.video_path)

        self.json_objects = []

        self.video_writer = None

        rospy.loginfo("%s init complete" % self.name)

    def add_object(self, timestamp, name, obj):
        row = [
            timestamp,
            name,
            obj
        ]
        rospy.loginfo(str(row))
        self.json_objects.append(row)
    
    def get_timestamp(self, ros_time=None):
        if ros_time is None:
            ros_time = rospy.Time.now()
        return float(ros_time.to_sec())

    def init_video_writer(self, shape):
        height, width = shape[0:2]
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        return cv2.VideoWriter(self.video_path, fourcc, self.video_fps, (width, height))

    def info_callback(self, msg):
        dict_msg = utils.msg_to_dict(msg)
        self.add_object(self.get_timestamp(), "camera_info", dict_msg)
        self.info_sub.unregister()  # only use the first message
        rospy.loginfo("Camera model loaded")
    
    def image_callback(self, msg):
        try:
            image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
            return
        if self.video_writer is None:
            self.video_writer = self.init_video_writer(image.shape)

        self.add_object(self.get_timestamp(msg.header.stamp), "frame", {"seq": msg.header.seq})
        if self.rotate == 1:
            image = cv2.rotate(image, cv2.ROTATE_90_CLOCKWISE)
        elif self.rotate == 2:
            image = cv2.rotate(image, cv2.ROTATE_180)
        elif self.rotate == 3:
            image = cv2.rotate(image, cv2.ROTATE_90_COUNTERCLOCKWISE)
        self.video_writer.write(image)

    def objs_generator(self):
        for row in self.json_objects:
            yield row

    def objs_to_json(self):
        stream_array = StreamArray(self.objs_generator())
        name = os.path.splitext(os.path.basename(self.video_path))[0]
        directory = os.path.dirname(self.video_path)
        path = os.path.join(directory, name + ".json")
        rospy.loginfo("Writing frame info to %s" % path)
        with open(path, 'w') as outfile:
            for chunk in json.JSONEncoder(indent=4).iterencode(stream_array):
                outfile.write(chunk)

    def run(self):
        rospy.spin()

    def shutdown_hook(self):
        self.objs_to_json()
        if self.video_writer is not None:
            rospy.loginfo("Writing video to %s" % self.video_path)
            self.video_writer.release()


if __name__ == "__main__":
    node = RecordToVideoFile()
    try:
        node.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        rospy.loginfo("Exiting %s node" % node.name)
