import os
from lxml import etree


def add_xml_prop(root, name, value=None):
    obj = etree.Element(name)
    if value is not None:
        if type(value) == bool:
            value = int(value)
        value = str(value)
        obj.text = value.encode()

    root.append(obj)
    return obj


def find_xml_prop(root, name):
    element = root.find(name)
    if isinstance(element, etree._Element):
        return element.text
    else:
        return element


class PascalVOCObject:
    def __init__(self):
        self.name = ""
        self.pose = "Unspecified"
        self.difficult = 0
        self.truncated = 0
        self.bndbox = [0, 0, 0, 0]  # [xmin, ymin, xmax, ymax]

    def constrain_bndbox(self, width, height):
        if self.bndbox[0] < 0:
            self.bndbox[0] = 0
        if self.bndbox[1] < 0:
            self.bndbox[1] = 0
        if self.bndbox[2] > width:
            self.bndbox[2] = width
        if self.bndbox[3] > height:
            self.bndbox[3] = height
        assert self.bndbox_is_ok()

    @classmethod
    def from_obj(cls, obj):
        self = cls()
        self.name = obj.name
        self.pose = obj.pose
        self.difficult = obj.difficult
        self.truncated = obj.truncated
        for index in range(len(self.bndbox)):
            self.bndbox[index] = obj.bndbox[index]

        assert self.bndbox_is_ok()

        return self

    @classmethod
    def from_element(cls, element):
        self = cls()

        self.name = find_xml_prop(element, "name")
        self.pose = find_xml_prop(element, "pose")
        self.truncated = self.parse_bndbox_prop(find_xml_prop(element, "truncated"))
        self.difficult = self.parse_bndbox_prop(find_xml_prop(element, "difficult"))
        bndbox = element.find("bndbox")
        xmin = self.parse_bndbox_prop(find_xml_prop(bndbox, "xmin"))
        ymin = self.parse_bndbox_prop(find_xml_prop(bndbox, "ymin"))
        xmax = self.parse_bndbox_prop(find_xml_prop(bndbox, "xmax"))
        ymax = self.parse_bndbox_prop(find_xml_prop(bndbox, "ymax"))
        self.bndbox = [xmin, ymin, xmax, ymax]

        assert self.bndbox_is_ok()

        return self

    @staticmethod
    def parse_bndbox_prop(x):
        try:
            return int(x)
        except ValueError:
            return int(float(x))

    @classmethod
    def from_open_images(cls, class_name, annotation, width, height):
        self = cls()

        self.name = class_name

        # XMin = float(annotation["XMin"])
        # YMin = float(annotation["YMin"])
        # XMax = float(annotation["XMax"])
        # YMax = float(annotation["YMax"])
        # self.bndbox[0] = int(XMin * width)
        # self.bndbox[1] = int(YMin * height)
        # self.bndbox[2] = int(XMax * width)
        # self.bndbox[3] = int(YMax * height)

        self.bndbox[0] = 0
        self.bndbox[1] = 0
        self.bndbox[2] = width
        self.bndbox[3] = height

        assert self.bndbox_is_ok()

        return self

    @classmethod
    def from_yolo(cls, class_name, yolo_bbox, width, height):
        self = cls()

        self.name = class_name

        cx = yolo_bbox[0]
        cy = yolo_bbox[1]
        w = yolo_bbox[2]
        h = yolo_bbox[3]

        x0 = cx - w / 2.0
        y0 = cy - h / 2.0
        x1 = x0 + w
        y1 = y0 + h

        self.bndbox[0] = int(x0 * width)  # xmin
        self.bndbox[1] = int(y0 * height)  # ymin
        self.bndbox[2] = int(x1 * width)  # xmax
        self.bndbox[3] = int(y1 * height)  # ymax

        assert self.bndbox_is_ok()

        self.truncated = self.is_truncated(width, height)

        return self

    def bndbox_is_ok(self):
        if self.bndbox[0] > self.bndbox[2]:
            return False
        if self.bndbox[1] > self.bndbox[3]:
            return False
        return True

    def is_truncated(self, width, height):
        if self.bndbox[0] <= 0:
            return True
        if self.bndbox[1] <= 0:
            return True
        if self.bndbox[2] >= width:
            return True
        if self.bndbox[3] >= height:
            return True
        return False

    def is_out_of_bounds(self, width, height):
        if self.bndbox[0] >= width:
            return True
        if self.bndbox[1] >= height:
            return True
        if self.bndbox[2] <= 0:
            return True
        if self.bndbox[3] <= 0:
            return True
        return False

    def to_xml(self, root=None):
        if root is None:
            root = etree.Element("object")
        add_xml_prop(root, "name", self.name)
        add_xml_prop(root, "pose", self.pose)
        add_xml_prop(root, "difficult", self.difficult)
        add_xml_prop(root, "truncated", self.truncated)
        bndbox = add_xml_prop(root, "bndbox")
        add_xml_prop(bndbox, "xmin", self.bndbox[0])
        add_xml_prop(bndbox, "ymin", self.bndbox[1])
        add_xml_prop(bndbox, "xmax", self.bndbox[2])
        add_xml_prop(bndbox, "ymax", self.bndbox[3])
        return root


class PascalVOCFrame:
    def __init__(self):
        self.folder = ""
        self.filename = ""
        self.path = ""
        self.frame_path = ""

        self.database = "Unknown"

        self.width = 0
        self.height = 0
        self.depth = 0

        self.segmented = 0
        self.verified = False

        self.objects = []

    @property
    def frame_id(self):
        return os.path.splitext(self.filename)[0]

    @classmethod
    def from_frame(cls, frame):
        self = cls()
        self.folder = frame.folder
        self.filename = frame.filename
        self.path = frame.path
        self.frame_path = frame.frame_path

        self.database = frame.database

        self.width = frame.width
        self.height = frame.height
        self.depth = frame.depth

        self.segmented = frame.segmented
        self.verified = frame.verified

        for obj in frame.objects:
            self.objects.append(PascalVOCObject.from_obj(obj))

        return self

    @classmethod
    def from_path(cls, path):
        self = cls()
        tree = etree.parse(path)
        verified = tree.getroot().get("verified")
        self.verified = verified == "yes"
        self.frame_path = path

        self.folder = find_xml_prop(tree, "folder")
        self.filename = find_xml_prop(tree, "filename")
        self.path = find_xml_prop(tree, "path")
        self.database = find_xml_prop(tree, "source/database")
        size = tree.find("size")
        self.width = int(find_xml_prop(size, "width"))
        self.height = int(find_xml_prop(size, "height"))
        self.depth = int(find_xml_prop(size, "depth"))
        self.segmented = int(find_xml_prop(tree, "segmented"))

        for element in tree.getroot().getchildren():
            if element.tag != "object":
                continue
            obj = PascalVOCObject.from_element(element)
            self.objects.append(obj)
        return self

    def set_path(self, path):
        self.folder = os.path.basename(os.path.dirname(path))
        self.filename = os.path.basename(path)
        self.path = path

    def add_object(self, obj):
        self.objects.append(obj)

    def to_xml(self):
        root = etree.Element("annotation")
        root.set("verified", "yes" if self.verified else "no")
        add_xml_prop(root, "folder", self.folder)
        add_xml_prop(root, "filename", self.filename)
        add_xml_prop(root, "path", self.path)
        db_src = add_xml_prop(root, "source")
        add_xml_prop(db_src, "database", self.database)

        size = add_xml_prop(root, "size")
        add_xml_prop(size, "width", self.width)
        add_xml_prop(size, "height", self.height)
        add_xml_prop(size, "depth", self.depth)

        add_xml_prop(root, "segmented", self.segmented)

        for obj in self.objects:
            root.append(obj.to_xml())
            # add_xml_prop(root, "object", obj.to_xml())

        return root

    def write(self, path=None):
        if path is None:
            name = os.path.splitext(self.filename)[0]
            dirname = os.path.dirname(self.path)
            path = os.path.join(dirname, name + ".xml")
        root = self.to_xml()
        et = etree.ElementTree(root)
        et.write(path, pretty_print=True)
        return root, path

    def pprint(self):
        print(etree.tostring(self.to_xml(), pretty_print=True).decode())
