import cv2
import tensorflow as tf
import time
from object_detection.utils import label_map_util
from object_detection.utils import visualization_utils as viz_utils
import numpy as np
from PIL import Image
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import warnings
# warnings.filterwarnings('ignore')   # Suppress Matplotlib warnings

from tensorflow.compat.v1 import ConfigProto
from tensorflow.compat.v1 import InteractiveSession

config = ConfigProto()
config.gpu_options.allow_growth = True
session = InteractiveSession(config=config)


detect_fn = None
category_index = None

def get_detect_fn():
    PATH_TO_LABELS = "annotations/label_map.pbtxt"
    category_index = label_map_util.create_category_index_from_labelmap(PATH_TO_LABELS,
                                                                        use_display_name=True)
    # PATH_TO_MODEL_DIR = "exported-models/dodobot_objects_faster_rcnn_resnet152_v1"
    # PATH_TO_MODEL_DIR = "exported-models/dodobot_objects_ssd_resnet50_v1_fpn"
    # PATH_TO_MODEL_DIR = "exported-models/faster_rcnn_resnet152_v1"
    # PATH_TO_MODEL_DIR = "exported-models/dodobot_objects_ssd_mobilenet_v2"
    PATH_TO_MODEL_DIR = "exported-models/dodobot_objects_efficientdet_d1"
    PATH_TO_SAVED_MODEL = PATH_TO_MODEL_DIR + "/saved_model"

    print('Loading model...', end='')
    start_time = time.time()

    # Load saved model and build the detection function
    detect_fn = tf.saved_model.load(PATH_TO_SAVED_MODEL)

    end_time = time.time()
    elapsed_time = end_time - start_time
    print('Done! Took {} seconds'.format(elapsed_time))

    return detect_fn, category_index


def load_image_into_numpy_array(path):
    """Load an image from file into a numpy array.

    Puts image into numpy array to feed into tensorflow graph.
    Note that by convention we put it into a numpy array with shape
    (height, width, channels), where channels=3 for RGB.

    Args:
      path: the file path to the image

    Returns:
      uint8 numpy array with shape (img_height, img_width, 3)
    """
    return np.array(Image.open(path))


def test_with_images():
    IMAGE_PATHS = [
        # "images/test/1606969821347451126.jpg",
        # "images/test/1606969838560394657.jpg",
        "images/infer/1606969821347451126.jpg",
        "images/infer/Screenshot from 2021-04-30 18-13-42.jpg",
        "images/infer/last_frame.jpg",
    ]

    for image_path in IMAGE_PATHS:

        print('Running inference for {}... '.format(image_path), end='')

        image_np = load_image_into_numpy_array(image_path)
        
        # Things to try:
        # Flip horizontally
        # image_np = np.fliplr(image_np).copy()

        # Convert image to grayscale
        # image_np = np.tile(
        #     np.mean(image_np, 2, keepdims=True), (1, 1, 3)).astype(np.uint8)

        # The input needs to be a tensor, convert it using `tf.convert_to_tensor`.
        input_tensor = tf.convert_to_tensor(image_np)
        print(image_np.shape)
        print(input_tensor.shape)
        # The model expects a batch of images, so add an axis with `tf.newaxis`.
        input_tensor = input_tensor[tf.newaxis, ...]

        # input_tensor = np.expand_dims(image_np, 0)
        detections = detect_fn(input_tensor)

        # All outputs are batches tensors.
        # Convert to numpy arrays, and take index [0] to remove the batch dimension.
        # We're only interested in the first num_detections.
        num_detections = int(detections.pop('num_detections'))
        detections = {key: value[0, :num_detections].numpy()
                    for key, value in detections.items()}
        detections['num_detections'] = num_detections

        # detection_classes should be ints.
        detections['detection_classes'] = detections['detection_classes'].astype(np.int64)

        image_np_with_detections = image_np.copy()

        viz_utils.visualize_boxes_and_labels_on_image_array(
            image_np_with_detections,
            detections['detection_boxes'],
            detections['detection_classes'],
            detections['detection_scores'],
            category_index,
            use_normalized_coordinates=True,
            max_boxes_to_draw=200,
            min_score_thresh=.30,
            agnostic_mode=False)

        plt.figure()
        plt.imshow(image_np_with_detections)
        print('Done')
    print("Showing images")
    plt.show()

def test_with_webcam():
    # path = "images/infer/infer_video-2021-05-21_10.01.16.mp4"
    path = "images/infer/infer_video-2021-05-21_10.59.19.mp4"
    # cap = cv2.VideoCapture(8)
    # # cap = cv2.VideoCapture(0)
    # cap.set(cv2.CAP_PROP_FRAME_WIDTH, 960)
    # cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 540)
    cap = cv2.VideoCapture(path)

    min_score = 0.3
    max_boxes = 10

    paused = False

    try:
        while True:
            key = cv2.waitKey(1)
            key &= 0xff
            key = chr(key)
            if key == 'q':
                break
            elif key == ' ':
                paused = not paused
                print("paused:", paused)
            if paused:
                time.sleep(0.05)
                continue

            # Read frame from camera
            ret, image_np = cap.read()
            if not ret:
                print("Failed to get frame")
                return
            image_np = cv2.cvtColor(image_np, cv2.COLOR_RGB2BGR)
            image_np = cv2.resize(image_np, (960, 540))

            # Expand dimensions since the model expects images to have shape: [1, None, None, 3]
            # image_np_expanded = np.expand_dims(image_np, axis=0)

            # Things to try:
            # Flip horizontally
            # image_np = np.fliplr(image_np).copy()

            # Convert image to grayscale
            # image_np = np.tile(
            #     np.mean(image_np, 2, keepdims=True), (1, 1, 3)).astype(np.uint8)
            
            # input_tensor = tf.convert_to_tensor(np.expand_dims(image_np, 0), dtype=tf.float32)
            # image_np = cv2.resize(image_np, (960, 540))
            input_tensor = tf.convert_to_tensor(image_np)
            input_tensor = input_tensor[tf.newaxis, ...]

            detections = detect_fn(input_tensor)

            label_id_offset = 1
            image_np_with_detections = image_np.copy()

            # All outputs are batches tensors.
            # Convert to numpy arrays, and take index [0] to remove the batch dimension.
            # We're only interested in the first num_detections.
            num_detections = int(detections.pop('num_detections'))
            detections = {key: value[0, :num_detections].numpy()
                        for key, value in detections.items()}
            detections['num_detections'] = num_detections

            # detection_classes should be ints.
            detections['detection_classes'] = detections['detection_classes'].astype(np.int64)

            # viz_utils.visualize_boxes_and_labels_on_image_array(
            #     image_np_with_detections,
            #     detections['detection_boxes'][0].numpy(),
            #     (detections['detection_classes'][0].numpy() + label_id_offset).astype(int),
            #     detections['detection_scores'][0].numpy(),
            #     category_index,
            #     use_normalized_coordinates=True,
            #     max_boxes_to_draw=20,
            #     min_score_thresh=.30,
            #     agnostic_mode=False)
            viz_utils.visualize_boxes_and_labels_on_image_array(
                image_np_with_detections,
                detections['detection_boxes'],
                detections['detection_classes'],
                detections['detection_scores'],
                category_index,
                use_normalized_coordinates=True,
                max_boxes_to_draw=max_boxes,
                min_score_thresh=min_score,
                agnostic_mode=False)

            image_np_with_detections = cv2.cvtColor(image_np_with_detections, cv2.COLOR_BGR2RGB)
            # Display output
            # cv2.imshow('object detection', cv2.resize(image_np_with_detections, (800, 600)))
            cv2.imshow('object detection', image_np_with_detections)
    finally:
        cap.release()
        cv2.destroyAllWindows()

def webcam():
    cap = cv2.VideoCapture(8)
    try:
        while True:
            # Read frame from camera
            ret, image_np = cap.read()
            cv2.imshow('object detection', image_np)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    finally:
        cap.release()
        cv2.destroyAllWindows()

# webcam()
detect_fn, category_index = get_detect_fn()
test_with_webcam()
# test_with_images()
