import os
from pathlib import Path
import shutil
from .dataset_builder import DatasetBuilder


class ClassifyDatasetBuilder(DatasetBuilder):
    def __init__(self, output_dir: Path, images_base_dir: Path, image_names: dict, test_ratio=0.15, train_ratio=0.85,
                 validation_ratio=0.0, dry_run=True):
        super(ClassifyDatasetBuilder, self).__init__(
            output_dir,
            test_ratio, train_ratio, validation_ratio,
            validation_name="val", dry_run=dry_run
        )
        self.images_base_dir = images_base_dir
        self.image_names = image_names
        self.reversed_image_names = self._reverse_image_map(self.image_names)

    def _reverse_image_map(self, image_names):
        reversed_image_names = {}
        for label, image_set in image_names.items():
            for image_filename in image_set:
                if image_filename in reversed_image_names:
                    raise ValueError("Non unique filename found in image mapping: %s" % image_filename)
                reversed_image_names[image_filename] = label
        return reversed_image_names

    def reset(self):
        self.reset_dir(self.output_dir, ".jpg", ".jpeg")

    def build(self):
        self.labels = sorted(list(self.image_names.keys()))
        image_sets = self.get_distributed_sets(self.image_names)

        test_count = len(image_sets[self.test_name])
        train_count = len(image_sets[self.train_name])
        validation_count = len(image_sets[self.validation_name])
        total_count = test_count + train_count + validation_count
        print(
            f"Total {total_count}:\n"
            f"\tTest: {test_count}\t{test_count / total_count:0.2f}\n"
            f"\tTrain: {train_count}\t{train_count / total_count:0.2f}\n"
            f"\tValidation: {validation_count}\t{validation_count / total_count:0.2f}"
        )

        for set_key in image_sets.keys():
            for label in self.labels:
                self.makedir(self.output_dir / set_key / label)

        for key, image_set in image_sets.items():
            for image_filename in image_set:
                image_label = self.reversed_image_names[image_filename]
                old_path = self.images_base_dir / image_filename
                new_path = self.output_dir / key / image_label / image_filename
                print("Copying image %s -> %s" % (old_path, new_path))
                if not self.dry_run:
                    shutil.copy(old_path, new_path)

        self.write_labels()
