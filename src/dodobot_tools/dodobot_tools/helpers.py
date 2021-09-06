
def get_msg_properties(msg):
    SKIP_PROPERTIES = ("serialize", "deserialize", "serialize_numpy", "deserialize_numpy")
    properties = {}
    for name in dir(msg):
        if name.startswith("_"):
            continue
        if name in SKIP_PROPERTIES:
            continue
        properties[name] = getattr(msg, name)
    return properties
