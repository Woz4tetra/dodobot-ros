
class YoloObject:
    def __init__(self):
        self.bndbox = [0.0, 0.0, 0.0, 0.0]  # [center x, center y, width, height] (normalized to image size)
        self.class_index = 0

    @classmethod
    def from_pascal_voc(cls, obj, class_mapping: list, image_width, image_height):
        self = cls()
        self.class_index = class_mapping.index(obj.name)
        box_width = (obj.bndbox[2] - obj.bndbox[0]) / image_width
        box_height = (obj.bndbox[3] - obj.bndbox[1]) / image_height
        cx = obj.bndbox[0] / image_width + box_width / 2.0
        cy = obj.bndbox[1] / image_height + box_height / 2.0

        self.bndbox[0] = cx
        self.bndbox[1] = cy
        self.bndbox[2] = box_width
        self.bndbox[3] = box_height

        return self

    def constrain_bndbox(self):
        if self.bndbox[0] < 0.0:
            self.bndbox[0] = 0.0
        if self.bndbox[1] < 0.0:
            self.bndbox[1] = 0.0
        if self.bndbox[2] > 1.0:
            self.bndbox[2] = 1.0
        if self.bndbox[3] > 1.0:
            self.bndbox[3] = 1.0
        assert self.bndbox_is_ok()

    def is_out_of_bounds(self):
        if self.bndbox[0] >= 1.0:
            return True
        if self.bndbox[1] >= 1.0:
            return True
        if self.bndbox[2] <= 0:
            return True
        if self.bndbox[3] <= 0:
            return True
        return False

    def bndbox_is_ok(self):
        if self.bndbox[0] > self.bndbox[2]:
            return False
        if self.bndbox[1] > self.bndbox[3]:
            return False
        if self.is_out_of_bounds():
            return False
        return True

    def to_txt(self):
        string = str(self.class_index) + " "
        string += " ".join(map(self._format_bndbox_element, self.bndbox))
        string += "\n"
        return string

    @staticmethod
    def _format_bndbox_element(x):
        return "%0.6f" % x


class YoloFrame:
    def __init__(self):
        self.objects = []

    def write(self, path):
        contents = ""
        for obj in self.objects:
            contents += obj.to_txt()
        with open(path, 'w') as file:
            file.write(contents)

    @classmethod
    def from_pascal_voc(cls, frame, class_mapping: list):
        self = cls()
        for obj in frame.objects:
            self.objects.append(YoloObject.from_pascal_voc(obj, class_mapping, frame.width, frame.height))
        return self
