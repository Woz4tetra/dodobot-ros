import os
import cv2
import math
import random
import numpy as np
from tj2_tools.training.pascal_voc import PascalVOCFrame, PascalVOCObject


def random_warp(offset, random_width):
    return offset + random.randint(-random_width // 2, random_width // 2)


def do_boxes_overlap(box, other):
    xmin, ymin, xmax, ymax = box
    o_xmin, o_ymin, o_xmax, o_ymax = other

    if xmin < o_xmax and xmax > o_xmin and ymax > o_ymin and ymin < o_ymax:
        return True
    else:
        return False


def check_bndbox(bndbox, other_bndboxes, w_bndbox, shape, min_percentage, max_percentage, lower_ratio, upper_ratio):
    # Bounding box is not ok if
    #   warped bounding box extends out of the image
    #   is smaller in width or height than the original bounding box by X * min_percentage
    #   is larger in width or height than the original bounding box by X * max_percentage
    #   new_width/new_height - old_width/old_height is less than lower_ratio or greater than upper_ratio 
    #   the new bounding box intersects with another bounding box in other_bndboxes
    w_xmin, w_ymin, w_xmax, w_ymax = w_bndbox
    xmin, ymin, xmax, ymax = bndbox
    height, width = shape[0:2]

    w_box_width = w_xmax - w_xmin
    box_width = xmax - xmin
    w_box_height = w_ymax - w_ymin
    box_height = ymax - ymin

    if w_xmin < 0 or w_xmin > w_xmax or w_xmax >= width:
        return False
    if w_box_width < box_width * min_percentage or w_box_width > box_width * max_percentage:
        return False
    if w_ymin < 0 or w_ymin > w_ymax or w_ymax >= height:
        return False
    if w_box_height < box_height * min_percentage or w_box_height > box_height * max_percentage:
        return False
    if not (lower_ratio < abs(box_width / box_height - w_box_width / w_box_height) < upper_ratio):
        return False
    for other_bndbox in other_bndboxes:
        if do_boxes_overlap(w_bndbox, other_bndbox):
            return False
    return True


def get_random_warp(bndbox, shape, random_width, random_angle):
    xmin, ymin, xmax, ymax = bndbox
    height, width = shape[0:2]

    assert random_width < min(width, height)

    w_xmin = random_warp(xmin, random_width)
    w_ymin = random_warp(ymin, random_width)
    w_xmax = random_warp(xmax, random_width)
    w_ymax = random_warp(ymax, random_width)

    random_trans_x = random.randint(-w_xmax, width - w_xmin)
    random_trans_y = random.randint(-w_ymax, height - w_ymin)

    src_triangle = np.array([[xmin, ymin], [xmax, ymin], [xmin, ymax]]).astype(np.float32)
    dst_triangle = np.array([[w_xmin, w_ymin], [w_xmax, w_ymin], [w_xmin, w_ymax]]).astype(np.float32)
    dst_triangle[:, 0] += random_trans_x
    dst_triangle[:, 1] += random_trans_y

    warp_mat = cv2.getAffineTransform(src_triangle, dst_triangle)

    if random_angle != 0.0:
        angle = 2 * (random.random() - 0.5) * random_angle
        rotate_matrix = cv2.getRotationMatrix2D(
            center=((xmax + xmin) // 2, (ymax + ymin) // 2),
            angle=math.degrees(angle),
            scale=1
        )

        warp_I = np.eye(3)
        warp_I[0:2] = warp_mat[0:2]
        rotate_I = np.eye(3)
        rotate_I[0:2] = rotate_matrix[0:2]

        warp_mat = np.dot(warp_I, rotate_I)[0:2]

    return warp_mat


def warp_bndbox(bndbox, warp_mat):
    xmin, ymin, xmax, ymax = bndbox
    pt1 = np.array([xmin, ymin, 1], dtype=np.int32)
    pt2 = np.array([xmax, ymin, 1], dtype=np.int32)
    pt3 = np.array([xmin, ymax, 1], dtype=np.int32)
    pt4 = np.array([xmax, ymax, 1], dtype=np.int32)
    pts = np.array([pt1, pt2, pt3, pt4])

    warp_pts = np.dot(pts, warp_mat.T)
    warp_pts = warp_pts[..., 0:4]
    warp_pts = warp_pts.reshape((-1, 1, 2))
    warp_pts = warp_pts.astype(np.int32)

    warp_pts = warp_pts[:, 0]
    # warp_bndbox = [
    #     warp_pts[0][0], warp_pts[0][1],
    #     warp_pts[1][0], warp_pts[1][1]
    # ]
    w_xmin = np.min(warp_pts[:, 0])
    w_xmax = np.max(warp_pts[:, 0])
    w_ymin = np.min(warp_pts[:, 1])
    w_ymax = np.max(warp_pts[:, 1])

    warp_bndbox = w_xmin, w_ymin, w_xmax, w_ymax
    return warp_bndbox


def randomly_transform_annotation(bndbox, other_bndboxes, image, object_mask_fn,
                                  random_width, lower_ratio, upper_ratio, random_angle,
                                  min_percentage, max_percentage):
    obj_mask = object_mask_fn(bndbox, image)

    if random_width <= 0:
        return image, obj_mask, bndbox

    width = image.shape[1]
    height = image.shape[0]
    size = width, height

    warp_mat = None
    w_bndbox = 0, 0, 0, 0
    bndbox_ok = False
    attempts = 0
    while not bndbox_ok:
        warp_mat = get_random_warp(bndbox, image.shape, random_width, random_angle)
        w_bndbox = warp_bndbox(bndbox, warp_mat)
        if check_bndbox(bndbox, other_bndboxes, w_bndbox, image.shape, min_percentage, max_percentage, lower_ratio,
                        upper_ratio):
            bndbox_ok = True
        attempts += 1
        if attempts >= 50000:
            raise RuntimeError("Failed to find warp bounding boxes that suit the parameters")

    warp_image = cv2.warpAffine(image, warp_mat, size)
    warp_mask = cv2.warpAffine(obj_mask, warp_mat, size)

    return warp_image, warp_mask, w_bndbox


def noisy_background(lower, upper, shape):
    return np.uint8(np.random.randint(lower, upper, shape))


def gauss_background(mean, sigma, shape):
    return np.uint8(np.clip(np.random.normal(mean, sigma, shape), 0, 255))


def gauss_foreground(image, mean, sigma, shape):
    return np.uint8(np.clip(image + np.random.normal(mean, sigma, shape), 0, 255))


def get_random_bgr(excluded_hues):
    random_h = random.choice([i for i in range(0, 256) if i not in excluded_hues])
    random_s = random.randint(0, 256)
    random_v = random.randint(0, 256)
    pixel = np.array([[[random_h, random_s, random_v]]]).astype(np.uint8)
    return cv2.cvtColor(pixel, cv2.COLOR_HSV2BGR).flatten()


def draw_random_shape(image, excluded_hues, included_shapes=None):
    height, width = image.shape[0:2]
    random_bgr = get_random_bgr(excluded_hues).tolist()

    if included_shapes is None:
        included_shapes = ["rect", "circ", "poly"]
    shape_type = random.choice(included_shapes)
    if shape_type == "rect":
        cx = random.randint(0, width)
        cy = random.randint(0, height)
        shape_rw = random.randint(0, width // 2) // 2
        shape_rh = random.randint(0, height // 2) // 2
        angle = random.random() * 2 * np.pi
        rotate_matrix = cv2.getRotationMatrix2D(center=(cx, cy), angle=math.degrees(angle), scale=1)
        points = np.array([
            [cx - shape_rw, cy - shape_rh, 1],
            [cx + shape_rw, cy - shape_rh, 1],
            [cx + shape_rw, cy + shape_rh, 1],
            [cx - shape_rw, cy + shape_rh, 1],
        ])

        points = points.dot(rotate_matrix.T)
        points = np.array([points]).astype(np.int32)
        return cv2.fillPoly(image, points, random_bgr)

    elif shape_type == "circ":
        cx = random.randint(0, width)
        cy = random.randint(0, height)
        radius = random.randint(min(width, height) // 6, min(width, height) // 2)
        return cv2.circle(image, (cx, cy), radius, random_bgr, cv2.FILLED)
    elif shape_type == "poly":
        points = []
        for _ in range(random.randint(3, 5)):
            x = random.randint(0, width)
            y = random.randint(0, height)
            points.append((x, y))
        points = np.array([points])
        return cv2.fillPoly(image, points, random_bgr)
    else:
        raise ValueError("Invalid shape type: %s" % shape_type)


def apply_random_shapes(background, num_shapes_min, num_shapes_max, shape_blur_min, shape_blur_max, included_shapes,
                        excluded_hues):
    for _ in range(random.randint(num_shapes_min, num_shapes_max)):
        random_shape = draw_random_shape(np.zeros_like(background), excluded_hues, included_shapes)
        ksize = random.randint(max(shape_blur_min // 2, 1), shape_blur_max // 2) * 2 - 1
        random_shape = cv2.GaussianBlur(random_shape, (ksize, ksize), 0)
        background = cv2.bitwise_or(background, random_shape)
    return background


def record_annotation(image, frame: PascalVOCFrame, directory, prefix="image"):
    image_path = record_image(image, directory, prefix)
    frame_path = os.path.splitext(image_path)[0] + ".xml"

    frame.set_path(os.path.abspath(image_path))
    frame.write(frame_path)

    return image_path, frame_path


def record_image(image, directory, resize=None, prefix="image"):
    if not os.path.isdir(directory):
        os.makedirs(directory)

    count = 0
    image_path = None

    while image_path is None:
        path = os.path.join(directory, "%s-%05d." % (prefix, count))
        image_path = path + "jpg"
        if os.path.isfile(image_path):
            image_path = None
            count += 1
    print("Writing to %s" % image_path)
    if resize is not None:
        image = cv2.resize(image, resize)
    cv2.imwrite(image_path, image)

    return image_path


def record_classify_paths(paths, directory):
    for label, image_paths in paths.items():
        label_path = os.path.join(directory, label + ".txt")
        with open(label_path, 'w') as file:
            file.write("\n".join(image_paths))


def read_classify_paths(directory):
    paths = {}
    for filename in os.listdir(directory):
        if not filename.lower().endswith(".txt"):
            continue
        label_path = os.path.join(directory, filename)
        with open(label_path) as file:
            contents = file.read()
        label = os.path.splitext(filename)[0]
        image_paths = contents.splitlines()
        paths[label] = image_paths
    return paths


def crop_to_annotations(image, frame):
    crops = {}
    for obj in frame.objects:
        if obj.name not in crops:
            crops[obj.name] = []
        xmin, ymin, xmax, ymax = obj.bndbox
        cropped_image = image[ymin: ymax, xmin: xmax]
        crops[obj.name].append(cropped_image)
    return crops


def crop_to_background(image, frame, min_box, max_box, num_backgrounds, background_name="background"):
    crops = {}
    other_bndboxes = []
    for obj in frame.objects:
        other_bndboxes.append(obj.bndbox)

    image_height, image_width = image.shape[:2]
    for count in range(num_backgrounds):
        background_box = None
        for _ in range(5000):
            box_width = random.randint(min_box[0], max_box[0])
            box_height = random.randint(min_box[1], max_box[1])
            box_x = random.randint(0, image_width - box_width)
            box_y = random.randint(0, image_height - box_height)
            bndbox = [box_x, box_y, box_x + box_width, box_y + box_height]
            does_overlap = False
            for other_bndbox in other_bndboxes:
                if do_boxes_overlap(bndbox, other_bndbox):
                    does_overlap = True
            if does_overlap:
                continue
            background_box = bndbox
            break
        if background_box is None:
            raise RuntimeError(
                "Failed to find a suitable background box for image size (%s, %s)" % (image_width, image_height))

        xmin, ymin, xmax, ymax = background_box
        cropped_image = image[ymin: ymax, xmin: xmax]
        if obj.name not in crops:
            crops[background_name] = []
        crops[background_name].append(cropped_image)
    return crops


def apply_objects_to_background(image, background, frame, object_mask_fn, kwargs):
    warp_mask = np.zeros(image.shape[0:2], dtype=np.uint8)
    warp_image = np.zeros_like(image)

    kwargs = kwargs.copy()
    if "obj_max_count" in kwargs:
        obj_max_count = kwargs.pop("obj_max_count")
    else:
        obj_max_count = None

    all_bndboxes = []
    first_warp = True
    new_objects = []

    objects_by_label = {}
    warps_by_label = {}
    objects_flattened = []
    for obj in frame.objects:
        if obj.name not in objects_by_label:
            objects_by_label[obj.name] = []
            if obj_max_count is None:
                max_num_warps = 1
            else:
                max_num_warps = obj_max_count[obj.name]
            warps_by_label[obj.name] = random.randint(1 if first_warp else 0, max_num_warps)
        first_warp = False
        num_warps = warps_by_label[obj.name]
        if len(objects_by_label[obj.name]) < num_warps:
            objects_by_label[obj.name].append(obj)
            objects_flattened.append(obj)

    for index, obj in enumerate(objects_flattened):
        warp_obj_image, warp_obj_mask, w_bndbox = randomly_transform_annotation(
            obj.bndbox, all_bndboxes, image, object_mask_fn, **kwargs)
        all_bndboxes.append(w_bndbox)
        new_object = PascalVOCObject.from_obj(obj)
        new_object.bndbox = w_bndbox
        new_objects.append(new_object)
        # obj_mask = get_object_mask(obj, image)
        # mask = cv2.bitwise_or(mask, obj_mask)
        warp_mask = cv2.bitwise_or(warp_mask, warp_obj_mask)
        warp_obj_image = cv2.bitwise_and(warp_obj_image, warp_obj_image, mask=warp_obj_mask)
        warp_image = cv2.bitwise_or(warp_image, warp_obj_image)
    frame.objects = new_objects

    background_masked = cv2.bitwise_and(background, background, mask=cv2.bitwise_not(warp_mask))
    image_masked = cv2.bitwise_and(warp_image, warp_image, mask=warp_mask)

    return cv2.bitwise_or(background_masked, image_masked)


def draw_bndbox(image, frame):
    draw_image = np.copy(image)
    for obj in frame.objects:
        # print(obj.bndbox)
        pt1 = obj.bndbox[0], obj.bndbox[1]
        pt2 = obj.bndbox[2], obj.bndbox[3]
        cv2.rectangle(draw_image, pt1, pt2, (0, 0, 255))
    return draw_image


def resize_proportional(image, resize_width):
    height, width = image.shape[0:2]
    resize_height = int(resize_width / width * height)
    return cv2.resize(image, (resize_width, resize_height))


def blank_background(shape):
    return np.zeros(shape).astype(np.uint8)


def random_color_background(shape, excluded_hues):
    random_bgr = get_random_bgr(excluded_hues)
    background = np.zeros(shape)
    background[..., 0:3] = random_bgr[0:3]
    return background.astype(np.uint8)


def debug_imshow(image, frame, height=540):
    image = draw_bndbox(image, frame)
    image = resize_proportional(image, height)

    cv2.imshow("image", image)
    key = cv2.waitKey(-1) & 0xff
    if key == ord('q'):
        quit()


def pad_bndbox(obj: PascalVOCObject, x_pad, y_pad, image_width, image_height):
    obj.bndbox[0] -= x_pad
    obj.bndbox[1] -= y_pad
    obj.bndbox[2] += x_pad
    obj.bndbox[3] += y_pad
    obj.constrain_bndbox(image_width, image_height)
