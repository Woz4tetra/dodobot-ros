import os
import csv
import shutil
import random
from PIL import Image

from pascal_voc import PascalVOCObject, PascalVOCFrame
from get_image_size import get_image_metadata


def read_classes(path):
    with open(path) as file:
        contents = file.read()

    index = 1
    classes = {0: "BACKGROUND"}
    for line in contents.splitlines():
        if len(line) == 0:
            continue
        classes[index] = line
        index += 1
    return classes


def read_open_image_classes(path):
    classes = {}
    with open(path) as file:
        reader = csv.reader(file)
        for row in reader:
            mid, class_name = row
            classes[mid] = class_name
    return classes


def read_bbox_yolo(classes, path):
    assert path.endswith(".txt"), path
    with open(path) as file:
        contents = file.read()

    bboxes = {}
    for line in contents.splitlines():
        if len(line) == 0:
            continue
        elements = line.split(" ")
        index = int(elements[0]) + 1
        name = classes[index]
        bbox = [float(x) for x in elements[1:]]  # YOLO format: center x, center y, width, height
        bboxes[name] = bbox
    return bboxes


def bbox_yolo_to_open_images(bbox):
    cx = bbox[0]
    cy = bbox[1]
    w = bbox[2]
    h = bbox[3]

    x0 = cx - w / 2.0
    y0 = cy - h / 2.0
    x1 = x0 + w
    y1 = y0 + h

    return [x0, x1, y0, y1]


def make_oi_obj(bbox, image_name, class_name):
    oi_bbox = bbox_yolo_to_open_images(bbox)
    open_images_label = {
        "ImageID": image_name,
        "Source": "xclick",
        "LabelName": "",
        "Confidence": 1.0,
        "XMin": oi_bbox[0],
        "XMax": oi_bbox[1],
        "YMin": oi_bbox[2],
        "YMax": oi_bbox[3],
        "IsOccluded": 0,
        "IsTruncated": 0,
        "IsGroupOf": 0,
        "IsDepiction": 0,
        "IsInside": 0,
        "id": "",
        "ClassName": class_name
    }


def get_voc_database(root_dirname, classes_path, width=960, height=540, depth=3):
    images = {}
    labels = {}

    classes = read_classes(classes_path)
    # open_images_classes = read_open_image_classes("class-descriptions-boxable.csv")

    for dirpath, dirnames, filenames in os.walk(root_dirname):
        for filename in filenames:
            name = os.path.splitext(filename)[0]
            path = os.path.join(dirpath, filename)
            if filename.endswith(".png"):
                images[name] = path
            elif filename.endswith(".txt"):
                labels[name] = path

    database = {}

    for image_name, path in images.items():
        if image_name in labels:
            bboxes = read_bbox_yolo(classes, labels[image_name])
        else:
            bboxes = {}

        frame = PascalVOCFrame()
        frame.set_path(path)
        frame.width = width
        frame.height = height
        frame.depth = depth

        for class_name, bbox in bboxes.items():
            obj = PascalVOCObject.from_yolo(class_name, bbox, width, height)
            frame.add_object(obj)

            if frame.folder != "assorted" and class_name not in frame.folder:
                print(f"WARNING: label '{class_name}' doesn't match folder name '{frame.folder}' in image {frame.path}")

        if len(bboxes) == 0:
            continue
        #     obj = PascalVOCObject.from_yolo("BACKGROUND", [0, 0, width, height], width, height)
        #     frame.add_object(obj)

        database[image_name] = frame

    return database


def make_voc_dataset(database, base_dir, copy_images=True):
    annotations_dir = os.path.join(base_dir, "Annotations")
    images_dir = os.path.join(base_dir, "JPEGImages")

    if not os.path.isdir(annotations_dir):
        os.makedirs(annotations_dir)
    if not os.path.isdir(images_dir):
        os.makedirs(images_dir)

    for image_name, frame in database.items():
        new_image_path = os.path.join(images_dir, image_name + ".jpg")
        annotation_path = os.path.join(annotations_dir, image_name + ".xml")
        old_image_path = frame.path
        frame.set_path(new_image_path)
        frame.write(annotation_path)
        print(f"Annotation: {annotation_path}")

        if copy_images:
            old_image_path_lower = old_image_path.lower()
            if old_image_path_lower.endswith(".png"):
                image = Image.open(old_image_path)
                rgb_image = image.convert("RGB")
                rgb_image.save(new_image_path)
            elif old_image_path_lower.endswith(".jpg"):
                shutil.copyfile(old_image_path, new_image_path)
            else:
                raise Exception(f"Invalid image extension: {old_image_path}")
            print(f"Image: {old_image_path} -> {new_image_path}")


def read_oi_annotations(training_type, path):
    annotations = []
    with open(path) as file:
        reader = csv.reader(file)
        header = next(reader)
        for row in reader:
            annotation = {}
            for index, element in enumerate(row):
                annotation[header[index]] = element
            annotation["TrainingType"] = training_type
            annotations.append(annotation)
    return annotations


def parse_oi_dataset(oi_dir):
    test_annotations = read_oi_annotations("test", os.path.join(oi_dir, "sub-test-annotations-bbox.csv"))
    train_annotations = read_oi_annotations("train", os.path.join(oi_dir, "sub-train-annotations-bbox.csv"))
    validation_annotations = read_oi_annotations("validation",
                                                 os.path.join(oi_dir, "sub-validation-annotations-bbox.csv"))

    annotations = []
    annotations.extend(test_annotations)
    annotations.extend(train_annotations)
    annotations.extend(validation_annotations)

    database = {}
    for annotation in annotations:
        image_id = annotation["ImageID"]
        image_path = os.path.join(oi_dir, annotation["TrainingType"], image_id + ".jpg")
        image_metadata = get_image_metadata(image_path)

        if image_id not in database:
            frame = PascalVOCFrame()
            frame.set_path(image_path)
            frame.width = image_metadata.width
            frame.height = image_metadata.height
            frame.depth = 3

            database[image_id] = frame
        else:
            frame = database[image_id]

        obj = PascalVOCObject.from_open_images("BACKGROUND", annotation, frame.width, frame.height)
        frame.add_object(obj)

    return database


def randomly_select(count, input_list, output_list):
    assert count <= len(input_list), f"{count} > {len(input_list)}"

    for _ in range(count):
        obj = random.choice(input_list)
        index = input_list.index(obj)
        output_list.append(input_list.pop(index))


def make_image_sets(database, base_dir):
    """
    for each class name, divide image ids into test, train, and validation by percentage randomly
    generate text file with image ids in image_sets_dir
    """
    image_sets_dir = os.path.join(base_dir, "ImageSets", "Main")
    train_path = os.path.join(image_sets_dir, "train.txt")
    val_path = os.path.join(image_sets_dir, "val.txt")
    trainval_path = os.path.join(image_sets_dir, "trainval.txt")
    test_path = os.path.join(image_sets_dir, "test.txt")

    if not os.path.isdir(image_sets_dir):
        os.makedirs(image_sets_dir)

    test_ratio = 0.15
    train_ratio = 0.8
    validation_ratio = 0.05
    assert abs(1.0 - (test_ratio + train_ratio + validation_ratio)) < 1E-8, test_ratio + train_ratio + validation_ratio

    label_unique_mapping = {}
    for image_id, frame in database.items():
        # use first object to characterize image for sorting
        if len(frame.objects) == 0:
            class_name = "BACKGROUND"
        else:
            class_name = frame.objects[0].name
        if class_name not in label_unique_mapping:
            label_unique_mapping[class_name] = []
        label_unique_mapping[class_name].append(image_id)

    test_image_ids = []
    train_image_ids = []
    validation_image_ids = []
    for class_name, image_ids in label_unique_mapping.items():
        class_count = len(image_ids)
        test_count = int(class_count * test_ratio)
        train_count = int(class_count * train_ratio)
        validation_count = int(class_count * validation_ratio)

        randomly_select(test_count, image_ids, test_image_ids)
        randomly_select(train_count, image_ids, train_image_ids)
        randomly_select(validation_count, image_ids, validation_image_ids)
        for _ in range(len(image_ids)):
            train_image_ids.append(image_ids.pop())

        assert len(image_ids) == 0, len(image_ids)

        print(
            f"Label {class_name} count: {class_count}\n"
            f"\tTest: {test_count}\t{len(test_image_ids)}\n"
            f"\tTrain: {train_count}\t{len(train_image_ids)}\n"
            f"\tValidation: {validation_count}\t{len(validation_image_ids)}"
        )

    test_count = len(test_image_ids)
    train_count = len(train_image_ids)
    validation_count = len(validation_image_ids)
    total_count = test_count + train_count + validation_count
    print(
        f"Total {total_count}:\n"
        f"\tTest: {test_count}\t{test_count / total_count:0.2f}\n"
        f"\tTrain: {train_count}\t{train_count / total_count:0.2f}\n"
        f"\tValidation: {validation_count}\t{validation_count / total_count:0.2f}"
    )
    with open(train_path, 'w') as file:
        file.write("\n".join(train_image_ids))
    with open(val_path, 'w') as file:
        file.write("\n".join(validation_image_ids))
    with open(trainval_path, 'w') as file:
        file.write("\n".join(train_image_ids))
        file.write("\n")
        file.write("\n".join(validation_image_ids))
    with open(test_path, 'w') as file:
        file.write("\n".join(test_image_ids))

def main():
    out_dir = "/home/ben/Documents/dodobot_voc_image_database"
    source_dir = "/home/ben/Documents/dodobot_objects"
    classes_path = "/home/ben/Documents/dodobot_objects/blue_cube/classes.txt"
    open_images_dir = "/home/ben/jetson-inference/python/training/detection/ssd/data/all"

    database = get_voc_database(source_dir, classes_path)
    make_voc_dataset(database, out_dir, copy_images=False)

    # oi_database = parse_oi_dataset(open_images_dir)
    # make_voc_dataset(oi_database, out_dir, copy_images=False)
    #
    # database.update(oi_database)

    make_image_sets(database, out_dir)


if __name__ == '__main__':
    main()
