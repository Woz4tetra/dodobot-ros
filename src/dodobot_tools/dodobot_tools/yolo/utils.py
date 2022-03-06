
def get_obj_id(class_index, class_count):
    return (class_count << 16) | (int(class_index))

def get_label(class_names, obj_id):
    class_index = obj_id & 0xffff
    class_count = obj_id >> 16

    label = class_names[class_index]
    return label, class_count


def read_class_names(class_names_path):
    with open(class_names_path) as file:
        lines = file.read().splitlines()
    names = []
    for line in lines:
        line = line.strip()
        if len(line) > 0:
            names.append(line)
    return names

