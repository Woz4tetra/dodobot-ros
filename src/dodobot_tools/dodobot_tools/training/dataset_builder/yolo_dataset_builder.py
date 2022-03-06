import os
from pathlib import Path
import shutil
from tj2_tools.training.pascal_voc import PascalVOCFrame
from tj2_tools.training.yolo import YoloFrame
from ..detect_collector import DetectCollector
from .dataset_builder import DatasetBuilder, BACKGROUND_LABEL
from ..get_image_size import get_image_size

ANNOTATIONS = "labels"
JPEGIMAGES = "images"


class YoloDatasetBuilder(DatasetBuilder):
    def __init__(self, output_dir: Path, image_collector: DetectCollector, labels: list, dry_run=True):
        super(YoloDatasetBuilder, self).__init__(output_dir, dry_run=dry_run)
        self.image_collector = image_collector
        self.frames = []
        self.frame_filenames = {}
        self.labels = labels

    def reset(self):
        # deletes all image and xml files under annotations and jpeg images respectively
        annotations_dir = self.output_dir / ANNOTATIONS
        images_dir = self.output_dir / JPEGIMAGES
        self.reset_dir(annotations_dir, ".txt")
        self.reset_dir(images_dir, ".jpg", ".jpeg")

    def build(self):
        self.frames = []
        for frame in self.image_collector.iter():
            self.frames.append(frame)
            self.copy_annotation(frame)
        self.write_labels()

    def write_labels(self):
        if BACKGROUND_LABEL in self.labels:
            self.labels.remove(BACKGROUND_LABEL)
        labels_path = self.output_dir / "classes.txt"
        print("Writing labels to %s" % labels_path)
        self.write_list(labels_path, self.labels)

    def copy_annotation(self, frame: PascalVOCFrame):
        if frame.filename not in self.frame_filenames:
            self.frame_filenames[frame.filename] = 0
        filename_count = self.frame_filenames[frame.filename]
        self.frame_filenames[frame.filename] += 1
        width, height = get_image_size(frame.path)
        frame.width = width
        frame.height = height

        annotation_dir = self.output_dir / ANNOTATIONS
        images_dir = self.output_dir / JPEGIMAGES
        self.makedir(annotation_dir)
        self.makedir(images_dir)
        new_image_path = images_dir / frame.filename
        if filename_count > 0:
            name = new_image_path.stem
            ext = new_image_path.suffix
            new_image_path = new_image_path.parent / ("%s-%05d%s" % (name, filename_count, ext))

        new_frame_path = annotation_dir / os.path.basename(frame.frame_path)
        if filename_count > 0:
            name = new_frame_path.stem
            ext = ".txt"
            new_frame_path = new_frame_path.parent / ("%s-%05d%s" % (name, filename_count, ext))
        else:
            name = new_frame_path.stem
            ext = ".txt"
            new_frame_path = new_frame_path.parent / ("%s%s" % (name, ext))

        print("Copying image %s -> %s%s" % (frame.path, new_image_path,
                                            (". Adding count: %05d" % filename_count) if filename_count > 0 else ""))
        if not self.dry_run:
            shutil.copy(frame.path, new_image_path)

        frame.set_path(new_image_path)
        print("Copying annotation %s -> %s" % (frame.frame_path, new_frame_path))
        frame.frame_path = str(new_frame_path.name)
        yolo_frame = YoloFrame.from_pascal_voc(frame, self.labels)
        if not self.dry_run:
            yolo_frame.write(str(new_frame_path))
