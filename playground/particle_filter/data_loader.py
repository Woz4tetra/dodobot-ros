import json
from scipy.spatial.transform import Rotation


def iter_bag(path):
    with open(path) as file:
        bag = json.load(file)

    for row in bag:
        yield row


def get_key(tree, key, default=None):
    key = key.split(".")
    try:
        return get_key_recurse(tree, key, 0)
    except KeyError:
        return default


def get_key_recurse(tree, key, index):
    subfield = key[index]
    if index + 1 == len(key):
        return tree[subfield]
    else:
        return get_key_recurse(tree[subfield], key, index + 1)


def yaw_from_quat(quat):
    r_mat = Rotation.from_quat([quat["x"], quat["y"], quat["z"], quat["w"]])
    return r_mat.as_euler("xyz")[2]


def header_to_stamp(header):
    return header["secs"] + header["nsecs"] * 1E-9

