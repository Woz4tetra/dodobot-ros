import os
import copy
import rospy
import rosbag
from datetime import datetime


def to_datestr(timestamp):
    return datetime.fromtimestamp(timestamp).strftime("%Y/%m/%d %H:%M:%S.%f")


def msg_to_dict(msg):
    msg_dict = {}
    
    for key, value in iter_msg(msg):
        assign_key(msg_dict, key, value)
    
    return msg_dict

def assign_key(tree, key, value, index=0):
    subfield = key[index]
    
    if len(key) == index + 1:
        tree[subfield] = value
        return
    
    if subfield not in tree:
        tree[subfield] = {}

    assign_key(tree[subfield], key, value, index + 1)
        
            

def iter_msg(msg, key=None):
    msg_type = type(msg)
    if key is None:
        key = []
    key = copy.copy(key)
    if hasattr(msg_type, "__slots__"):
        key.append(None)
        for slot in msg_type.__slots__:
            key[-1] = slot
            val = getattr(msg, slot)
            for result in iter_msg(val, key):
                yield result

    elif is_list(msg):
        key.append(None)
        for index, attr in enumerate(msg):
            key[-1] = index
            for result in iter_msg(attr, key):
                yield result

    else:
        yield key, msg

def get_output_path(options, topic_name=None):
    bag_path = options.path
    bag_filename = os.path.basename(bag_path)
    bag_name = os.path.splitext(bag_filename)[0]
    
    if topic_name is None:
        output_filename = bag_name
    else:
        topic_name = topic_name.replace("/", "-")
        output_filename = "%s-%s" % (bag_name, topic_name)
    
    return os.path.join(options.output_dir, output_filename)

def get_topic_list(path):
    bag = rosbag.Bag(path)
    
    # index 0 is msg type mapped to msg hash
    # index 1 is topic name mapped to TopicTuple (contains msg type, message count, etc)
    topic_tuples = bag.get_type_and_topic_info()[1]  # type: dict[TopicTuple]
    topics = list(topic_tuples.keys())
    return topics


def enumerate_bag(options, fname):
    try:
        bag = rosbag.Bag(fname)
        stime = None
        if options.start_time:
            stime = rospy.Time(options.start_time)
        etime = None
        if options.end_time:
            etime = rospy.Time(options.end_time)
    except Exception as e:
        rospy.logfatal('failed to load bag file: %s', e)
        exit(1)
    
    try:
        for topic, msg, time in bag.read_messages(topics=options.topic_names, start_time=stime, end_time=etime):
            yield topic, msg, time
    except Exception as e:
        rospy.logwarn("failed to parse bag: %s", e)
    finally:
        bag.close()


def is_list(obj):
    return type(obj) == list or type(obj) == tuple
