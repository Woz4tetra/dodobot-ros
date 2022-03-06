import os
from pathlib import Path
import shutil
from tj2_tools.training.pascal_voc import PascalVOCFrame
from ..detect_collector import DetectCollector
from .dataset_builder import DatasetBuilder, BACKGROUND_LABEL

ANNOTATIONS = "Annotations"
IMAGESETS = "ImageSets/Main"
JPEGIMAGES = "JPEGImages"


class DetectDatasetBuilder(DatasetBuilder):
    def __init__(self, output_dir: Path, image_collector: DetectCollector, test_ratio=0.15, train_ratio=0.85,
                 validation_ratio=0.0, dry_run=True):
        super(DetectDatasetBuilder, self).__init__(output_dir, test_ratio, train_ratio, validation_ratio,
                                                   dry_run=dry_run)
        self.image_collector = image_collector
        self.frames = []
        self.trainval_name = self.train_name + "val"
        self.frame_filenames = {}

    def reset(self):
        # deletes all image and xml files under annotations and jpeg images respectively
        annotations_dir = self.output_dir / ANNOTATIONS
        images_dir = self.output_dir / JPEGIMAGES
        self.reset_dir(annotations_dir, ".xml")
        self.reset_dir(images_dir, ".jpg", ".jpeg")

    def build(self):
        self.frames = []
        label_set = set()
        for frame in self.image_collector.iter():
            self.frames.append(frame)
            self.copy_annotation(frame)
            for obj in frame.objects:
                label_set.add(obj.name)

        self.labels = sorted(list(label_set))

        self.build_training_sets()
        self.write_labels()

    def copy_annotation(self, frame: PascalVOCFrame):
        if frame.filename not in self.frame_filenames:
            self.frame_filenames[frame.filename] = 0
        filename_count = self.frame_filenames[frame.filename]
        self.frame_filenames[frame.filename] += 1

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
            ext = new_frame_path.suffix
            new_frame_path = new_frame_path.parent / ("%s-%05d%s" % (name, filename_count, ext))

        print("Copying image %s -> %s%s" % (frame.path, new_image_path,
                                            (". Adding count: %05d" % filename_count) if filename_count > 0 else ""))
        if not self.dry_run:
            shutil.copy(frame.path, new_image_path)

        frame.set_path(new_image_path)
        print("Copying annotation %s -> %s" % (frame.frame_path, new_frame_path))
        frame.frame_path = str(new_frame_path.name)
        if not self.dry_run:
            frame.write(str(new_frame_path))

    def map_label_to_frames(self):
        # group frames by the first label in the annotation
        # keys are the class label
        # values are corresponding index in self.frames
        label_unique_mapping = {}
        for index, frame in enumerate(self.frames):
            # use first object to characterize image for sorting
            if len(frame.objects) == 0:
                class_name = BACKGROUND_LABEL
            else:
                class_name = frame.objects[0].name
            if class_name not in label_unique_mapping:
                label_unique_mapping[class_name] = []
            label_unique_mapping[class_name].append(index)
        return label_unique_mapping

    def frame_index_to_anno_path(self, index, relative=True):
        if relative:
            path = self.frames[index].frame_path
        else:
            path = os.path.abspath(self.frames[index].frame_path)
        return os.path.splitext(path)[0]

    def build_training_sets(self):
        imageset_dir = self.output_dir / IMAGESETS
        self.makedir(imageset_dir)

        set_paths = {
            self.test_name: imageset_dir / "test.txt",
            self.train_name: imageset_dir / "train.txt",
            self.validation_name: imageset_dir / "val.txt",
            self.trainval_name: imageset_dir / "trainval.txt",
        }

        label_unique_mapping = self.map_label_to_frames()
        image_indices = self.get_distributed_sets(label_unique_mapping)

        test_count = len(image_indices[self.test_name])
        train_count = len(image_indices[self.train_name])
        validation_count = len(image_indices[self.validation_name])
        total_count = test_count + train_count + validation_count
        print(
            f"Total {total_count}:\n"
            f"\tTest: {test_count}\t{test_count / total_count:0.2f}\n"
            f"\tTrain: {train_count}\t{train_count / total_count:0.2f}\n"
            f"\tValidation: {validation_count}\t{validation_count / total_count:0.2f}"
        )

        set_anno_paths = {}
        for key, set_indices in image_indices.items():
            output_path = set_paths[key]
            anno_paths = [self.frame_index_to_anno_path(index, relative=True) for index in set_indices]
            set_anno_paths[key] = anno_paths
            print("Writing %s image set with %s annotations: %s" % (key, len(anno_paths), output_path))
            self.write_list(output_path, anno_paths)

        # trainval combines train and validation sets
        trainval_set = set_anno_paths[self.train_name] + set_anno_paths[self.validation_name]
        trainval_path = set_paths[self.trainval_name]
        print("Writing trainval image set with %s annotations: %s" % (len(trainval_set), trainval_path))
        self.write_list(trainval_path, trainval_set)
