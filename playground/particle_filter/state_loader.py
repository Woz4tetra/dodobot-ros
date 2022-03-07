import os
import pickle
from state import State, OBJECT_NAMES
from data_loader import iter_bag, get_key, header_to_stamp, yaw_from_quat


def get_pkl_path(path):
    pkl_dir = os.path.dirname(path)
    pkl_name = os.path.splitext(os.path.basename(path))[0]
    pkl_filename = pkl_name + ".pkl"
    pkl_path = os.path.join(pkl_dir, pkl_filename)
    return pkl_path


def make_pkl(path):
    states = []
    print("Creating pickle from %s" % path)

    for timestamp, topic, msg in iter_bag(path):
        if topic == "/dodobot/cmd_vel":
            state = State()
            state.type = "cmd_vel"
            state.stamp = timestamp
            state.vx = get_key(msg, "linear.x")
            state.vy = get_key(msg, "linear.y")
            state.vt = get_key(msg, "angular.z")
            states.append(state)

        elif topic == "/dodobot/odom":
            state = State()
            state.type = "odom"
            state.stamp = header_to_stamp(get_key(msg, "header.stamp"))
            state.x = get_key(msg, "pose.pose.position.x")
            state.y = get_key(msg, "pose.pose.position.y")
            state.t = yaw_from_quat(get_key(msg, "pose.pose.orientation"))

            state.vx = get_key(msg, "twist.twist.linear.x")
            state.vy = get_key(msg, "twist.twist.linear.y")
            state.vt = get_key(msg, "twist.twist.angular.z")

            states.append(state)

        elif topic == "/dodobot/detections":
            stamp = header_to_stamp(get_key(msg, "header.stamp"))
            detections = get_key(msg, "detections")
            for detection in detections.values():
                object_id = get_key(detection, "results.0.id")
                object_label = OBJECT_NAMES[object_id]

                # if object_label == "blue_cut_sphere":
                state = State()
                state.type = object_label
                state.stamp = stamp
                state.x = get_key(detection, "results.0.pose.pose.position.x")
                state.y = get_key(detection, "results.0.pose.pose.position.y")
                state.z = get_key(detection, "results.0.pose.pose.position.z")

                states.append(state)

    pickle_path = get_pkl_path(path)
    with open(pickle_path, 'wb') as file:
        pickle.dump(states, file)
    print("Pickle created: %s" % pickle_path)

    return states


def read_pkl(path, repickle=False):
    pkl_path = get_pkl_path(path)
    if not os.path.isfile(pkl_path) or repickle:
        return make_pkl(path)
    else:
        with open(pkl_path, 'rb') as file:
            return pickle.load(file)
