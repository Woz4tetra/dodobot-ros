import sys
from pathlib import Path

sys.path.insert(0, "../../src/db_tools")

from db_tools.training.detect_collector import DetectCollector
from db_tools.training.dataset_builder.detect_dataset_builder import DetectDatasetBuilder
from db_tools.training.dataset_builder.yolo_dataset_builder import YoloDatasetBuilder


def format_detect_dataset():
    base_dir = "resources/labeled-images"

    collector = DetectCollector(base_dir)
    # dataset = DetectDatasetBuilder(Path("outputs/cargo_2022_voc_dataset"), collector, dry_run=True)
    dataset = YoloDatasetBuilder(Path("outputs/cargo_2022_yolo_dataset"), collector, ["cargo_red", "cargo_blue"],
                                 dry_run=False)
    dataset.reset()
    dataset.build()


if __name__ == '__main__':
    format_detect_dataset()
