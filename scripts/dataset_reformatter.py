import os
import csv


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


def read_bbox(classes, path):
    assert path.endswith(".txt"), path
    with open(path) as file:
        contents = file.read()

    bboxes = {}
    for line in contents.splitlines():
        if len(line) == 0:
            continue
        elements = line.split(" ")
        index = int(elements[0])
        name = classes[index]
        bbox = [float(x) for x in elements[1:]]
        bboxes[name] = bbox
    return bboxes


def main():
    root_dirname = "labeled"
    images = {}
    labels = {}

    classes = read_classes("labeled/blue_cube/classes.txt")
    open_images_classes = read_open_image_classes("class-descriptions-boxable.csv")

    for dirpath, dirnames, filenames in os.walk(root_dirname):
        for filename in filenames:
            name = os.path.splitext(filename)[0]
            path = os.path.join(dirpath, filename)
            if filename.endswith(".png"):
                images[name] = path
            elif filename.endswith(".txt"):
                labels[name] = path

    for name, path in images.items():
        if name in labels:
            bboxes = read_bbox(classes, labels[name])
        else:
            bboxes = {}


        open_images_label = {
            "ImageID": name,
            "Source": "xclick",
            "LabelName": "",
            "Confidence": 1.0,
            "XMin": "",
            "XMax": "",
            "YMin": "",
            "YMax": "",
            "IsOccluded": "",
            "IsTruncated": "",
            "IsGroupOf": "",
            "IsDepiction": "",
            "IsInside": "",
            "id": "",
            "ClassName": ""
        }


if __name__ == '__main__':
    main()
