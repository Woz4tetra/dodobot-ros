#!/usr/bin/python
# -*- coding: utf-8 -*-
import sys
import rospy

from PyQt4 import QtGui
from SimplePyQtGUIKit import SimplePyQtGUIKit

from optparse import OptionParser
from datetime import datetime

import utils

from rosbag_to_csv import bag_to_csv
from rosbag_to_json import bag_to_json

ALLOWED_TYPES = ["csv", "json"]


def main(options):
    gui_enabled = not options.no_gui
    if gui_enabled:
        app = QtGui.QApplication(sys.argv)
    else:
        app = None

    if not gui_enabled:
        rospy.loginfo("GUI disabled. Assuming all topics are wanted")
        options.all_topics = True

    if len(options.path) == 0:
        if gui_enabled:
            files = SimplePyQtGUIKit.GetFilePath("Select bag file", "*bag")
            if len(files) < 1:
                rospy.logerr("Please select a bag file")
                sys.exit()
            options.path = files[0]
        else:
            rospy.logerr("Please provide a bag path with --path")
            sys.exit(1)
    
    if len(options.output_dir) == 0:
        if gui_enabled:
            dirs = SimplePyQtGUIKit.GetDirectory("Select output directory", ".")
            if len(dirs) < 1:
                rospy.logerr("Please select an output directory")
                sys.exit()
            options.output_dir = dirs[0]
        else:
            rospy.logerr("Please provide an output path with --output_dir")
            sys.exit(1)

    topics = utils.get_topic_list(options.path)
    options.topic_names = []

    if options.all_topics:
        options.topic_names = topics
    elif gui_enabled:
        selected = SimplePyQtGUIKit.GetCheckButtonSelect(topics, app=app, msg="Select topics to convert")
        for topic_name, is_selected in selected.items():
            if is_selected:
                options.topic_names.append(topic_name)

    if len(options.topic_names) == 0:
        rospy.logerr("Please select topics")
        sys.exit()

    rospy.loginfo("Converting to %s" % options.output_type)
    if options.output_type == "csv":
        bag_to_csv(options)
    elif options.output_type == "json":
        bag_to_json(options)
    else:
        message = "Invalid output type: %s. Allowed types are: %s" % (options.output, ", ".join(ALLOWED_TYPES))
        rospy.logerr(message)
        if gui_enabled:
            QtGui.QMessageBox.information(QtGui.QWidget(), "Message", message)
        sys.exit()

    if gui_enabled:
        QtGui.QMessageBox.information(QtGui.QWidget(), "Message", "Finished conversion!")

if __name__ == '__main__':
    rospy.loginfo("rosbag_to_file")

    rospy.init_node('rosbag_to_file', anonymous=True)
    parser = OptionParser(usage="%prog [options] bagfile")
    parser.add_option("-a", "--all", dest="all_topics",
            action="store_true",
            help="exports all topics", default=False)
    parser.add_option("-t", "--topic", dest="topic_names",
            action="append", help="white list topic names", metavar="TOPIC_NAME")
    parser.add_option("-s", "--start-time", dest="start_time",
                      help="start time of bagfile", type="float")
    parser.add_option("-e", "--end-time", dest="end_time",
                      help="end time of bagfile", type="float")
    parser.add_option("-n", "--no-header", dest="header",
                      action="store_false", default=True,
                      help="no header / flatten array value")
    parser.add_option("-p", "--path", dest="path", default="",
                      help="bag file to parse")
    parser.add_option("--nogui", dest="no_gui", action="store_true",
                      help="disable GUI for remote connections")
    parser.add_option("--type", dest="output_type", choices=ALLOWED_TYPES,
                      default="json", help="output file type")
    parser.add_option("-o", "--output_dir", dest="output_dir", default="",
                      help="output dir")
    (options, args) = parser.parse_args()


    main(options)
