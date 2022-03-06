import os
import cv2
from tj2_tools.training.get_image_size import get_image_metadata
from tj2_tools.training.pascal_voc import PascalVOCFrame, PascalVOCObject


class DetectCollector:
    def __init__(self, base_dir):
        self.base_dir = base_dir
        self.frames = []
        self.images = []
        self.metadata = []

        self.load_annotations()

    def load_annotations(self):
        _all_filenames = set()
        for dirpath, dirnames, filenames in os.walk(self.base_dir):
            for filename in filenames:
                if filename.endswith(".xml"):
                    path = os.path.join(dirpath, filename)
                    frame = PascalVOCFrame.from_path(path)
                    if os.path.isfile(frame.path):
                        image_path = frame.path
                    else:
                        image_path = os.path.join(dirpath, frame.filename)
                        frame.set_path(image_path)
                    if not os.path.isfile(image_path):
                        raise FileNotFoundError("Image file corresponding to '%s' annotation could not be found" % path)
                    image_metadata = get_image_metadata(image_path)

                    self.frames.append(frame)
                    self.images.append(None)
                    self.metadata.append(image_metadata)

    def _load_image(self, index):
        frame = self.frames[index]
        path = os.path.join(self.base_dir, frame.filename)
        image = cv2.imread(path)
        self.images[index] = image

    def iter(self, include_image=False, include_metadata=False):
        assert len(self.frames) == len(self.images) == len(self.metadata)
        for index in range(len(self.frames)):
            frame = self.frames[index]
            result = [frame]
            if include_image:
                image = self.images[index]
                if image is None:
                    self._load_image(index)
                    image = self.images[index]
                result.append(image)
            if include_metadata:
                image_metadata = self.metadata[index]
                result.append(image_metadata)
            if len(result) == 1:
                yield result[0]
            else:
                yield tuple(result)
