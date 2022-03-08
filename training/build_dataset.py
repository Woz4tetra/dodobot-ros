import os
import shutil
from PIL import Image


def make_dirs(directory, dry_run=False):
    if not os.path.isdir(directory):
        print("Making directory '%s'" % directory)
        if not dry_run:
            os.makedirs(directory)


def copy_png_to_jpg(input_path, output_path, dry_run=False):
    print("Copying image %s -> %s" % (input_path, output_path))
    if not dry_run:
        image = Image.open(input_path)
        image.save(output_path)

def copy_file(input_path, output_path, dry_run=False):
    print("Copying file %s -> %s" % (input_path, output_path))
    if not dry_run:
        shutil.copy(input_path, output_path)


def format_detect_dataset():
    input_dir = "resources/labeled-images"
    output_dir = "resources/yolo-dataset"
    labels = [
        "cosmo_cube",
        "blue_cut_sphere",
        "red_cut_sphere",
        "blue_low_bin",
        "red_low_bin",
        "blue_cube",
        "red_cube",
    ]
    dry_run = False

    label_mapping = {}
    for dirpath, dirnames, filenames in os.walk(input_dir):
        for filename in filenames:
            name, extension = os.path.splitext(filename)
            path = os.path.join(dirpath, filename)
            if name not in label_mapping:
                label_mapping[name] = ["", ""]

            if extension == ".png":
                if len(label_mapping[name][0]) != 0 and len(label_mapping[name][1]) != 0:
                    raise ValueError("%s already has an image" % name)
                label_mapping[name][0] = path
            elif extension == ".txt":
                if len(label_mapping[name][0]) != 0 and len(label_mapping[name][1]) != 0:
                    raise ValueError("%s already has a label" % name)
                label_mapping[name][1] = path
    filtered_mapping = {}
    for name, paths in label_mapping.items():
        if len(paths[0]) == 0 or len(paths[1]) == 0:
            continue
        filtered_mapping[name] = paths
    
    output_images_path = os.path.join(output_dir, "images")
    output_labels_path = os.path.join(output_dir, "labels")
    classes_path = os.path.join(output_dir, "classes.txt")
    make_dirs(output_images_path, dry_run)
    make_dirs(output_labels_path, dry_run)

    for name, (image_path, label_path) in filtered_mapping.items():
        output_image_path = os.path.join(output_images_path, name + ".jpg")
        output_label_path = os.path.join(output_labels_path, name + ".txt")
        copy_png_to_jpg(image_path, output_image_path, dry_run)
        copy_file(label_path, output_label_path, dry_run)

    if not dry_run:
        with open(classes_path, 'w') as file:
            file.write("\n".join(labels) + "\n")

if __name__ == '__main__':
    format_detect_dataset()
