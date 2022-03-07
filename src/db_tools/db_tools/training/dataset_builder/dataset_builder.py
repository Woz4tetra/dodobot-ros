import os
import random
from pathlib import Path

BACKGROUND_LABEL = "BACKGROUND"
LABELS = "labels.txt"


class DatasetBuilder:
    def __init__(self, output_dir: Path, test_ratio=0.15, train_ratio=0.85, validation_ratio=0.0,
                 test_name="test", train_name="train", validation_name="validation", dry_run=True):
        self.dry_run = dry_run
        self.output_dir = output_dir
        self.makedir(self.output_dir)
        self.labels = []
        self.test_ratio = test_ratio
        self.train_ratio = train_ratio
        self.validation_ratio = validation_ratio
        self.test_name = test_name
        self.train_name = train_name
        self.validation_name = validation_name

        assert self.check_ratios(self.test_ratio, self.train_ratio, self.validation_ratio), \
            self.test_ratio + self.train_ratio + self.validation_ratio

        self.ratios = {
            self.test_name: self.test_ratio,
            self.train_name: self.train_ratio,
            self.validation_name: self.validation_ratio
        }

    def reset_dir(self, directory: Path, *extensions):
        # deletes all image and xml files under annotations and jpeg images respectively

        for dirpath, dirnames, filenames in os.walk(directory):
            for filename in filenames:
                for extension in extensions:
                    if filename.lower().endswith(extension):
                        path = Path(dirpath) / filename
                        print("Deleting %s" % path)
                        if self.dry_run:
                            continue
                        os.remove(path)

    def makedir(self, directory: Path):
        if self.dry_run:
            return
        if not directory.is_dir():
            os.makedirs(directory)

    def build(self):
        pass

    def get_distributed_sets(self, label_to_identifier_map):
        """
        Create a dictionary of annotations split between test, train, and validation
        :param label_to_identifier_map: a dictionary mapping a unique label to an annotation identifier.
            Can be an index in a list or a filename for example
        :return: dict
        """
        image_sets = {
            self.test_name: [],
            self.train_name: [],
            self.validation_name: []
        }
        for class_name, frame_identifiers in label_to_identifier_map.items():
            class_count = len(frame_identifiers)
            counts = {
                self.test_name: int(class_count * self.test_ratio),
                self.train_name: int(class_count * self.train_ratio),
                self.validation_name: int(class_count * self.validation_ratio)
            }

            for key, set_count in counts.items():
                self.randomly_select(set_count, frame_identifiers, image_sets[key])
            for _ in range(len(frame_identifiers)):  # dump any remaining frames into the training set
                image_sets["train"].append(frame_identifiers.pop())

            assert len(frame_identifiers) == 0, len(frame_identifiers)
            for key, indices in image_sets.items():
                if self.ratios[key] > 0.0:
                    assert len(indices) > 0, "%s is empty!" % key

            print(
                f"Label {class_name} count: {class_count}\n"
                f"\tTest: {counts[self.test_name]}\t{len(image_sets[self.test_name])}\n"
                f"\tTrain: {counts[self.train_name]}\t{len(image_sets[self.train_name])}\n"
                f"\tValidation: {counts[self.validation_name]}\t{len(image_sets[self.validation_name])}"
            )
        return image_sets

    def write_labels(self):
        if BACKGROUND_LABEL in self.labels:
            self.labels.remove(BACKGROUND_LABEL)
        labels_path = self.output_dir / LABELS
        print("Writing labels to %s" % labels_path)
        self.write_list(labels_path, self.labels)

    def check_ratios(self, test_ratio, train_ratio, validation_ratio):
        return abs(1.0 - (test_ratio + train_ratio + validation_ratio)) < 1E-8

    def randomly_select(self, count, input_list, output_list):
        assert count <= len(input_list), f"{count} > {len(input_list)}"

        for _ in range(count):
            obj = random.choice(input_list)
            index = input_list.index(obj)
            output_list.append(input_list.pop(index))

    def write_list(self, path, l):
        if not self.dry_run:
            with open(path, 'w') as file:
                file.write("\n".join(l))
