import time
import torch
import numpy as np
from torch.backends import cudnn
from yolov5.models.common import DetectMultiBackend
from yolov5.utils.torch_utils import select_device
from yolov5.utils.general import non_max_suppression, scale_coords, xyxy2xywh
from yolov5.utils.plots import Annotator, colors
from yolov5.utils.augmentations import letterbox
from .utils import get_label, get_obj_id


class YoloDetector:
    def __init__(self, device, model_path, image_width, image_height, confidence_threshold=0.25, nms_iou_threshold=0.45,
                 max_detections=1000, report_loop_times=False, publish_overlay=False):
        self.model_device = device
        self.model_path = model_path
        self.report_loop_times = report_loop_times
        self.publish_overlay = publish_overlay
        self.image_width = image_width
        self.image_height = image_height
        self.max_detections = max_detections
        self.confidence_threshold = confidence_threshold
        self.nms_iou_threshold = nms_iou_threshold

        self.classes_filter = None
        self.half = False  # flag for whether to use half or full precision floats
        self.augment = False  # augmented inference
        self.agnostic_nms = False  # class-agnostic NMS
        self.overlay_line_thickness = 3  # bounding box thickness (pixels)

        self.selected_model_device = select_device(self.model_device)
        self.model = DetectMultiBackend(self.model_path, device=self.selected_model_device, dnn=False)

        self.stride = self.model.stride
        self.class_names = self.model.names
        pt = self.model.pt
        jit = self.model.jit
        onnx = self.model.onnx
        engine = self.model.engine

        # FP16 supported on limited backends with CUDA
        self.half &= (pt or jit or onnx or engine) and self.selected_model_device.type != 'cpu'
        if pt or jit:
            self.model.model.half() if self.half else self.model.model.float()

        cudnn.benchmark = True  # set True to speed up constant image size inference

        self.timing_report = ""

        # Run inference
        self.image_size = (self.image_width, self.image_height)
        self.model.warmup(imgsz=(1, 3, *self.image_size), half=self.half)  # warmup

    def detect(self, image):
        t_start = time.time()
        # Padded resize
        trans_image = letterbox(image, self.image_size, stride=self.stride, auto=True)[0]

        # Convert
        trans_image = trans_image.transpose((2, 0, 1))[::-1]  # HWC to CHW, BGR to RGB
        trans_image = np.ascontiguousarray(trans_image)

        torch_image = torch.from_numpy(trans_image).to(self.selected_model_device)
        torch_image = torch_image.half() if self.half else torch_image.float()  # uint8 to fp16/32
        torch_image /= 255  # 0 - 255 to 0.0 - 1.0
        if len(torch_image.shape) == 3:
            torch_image = torch_image[None]  # expand for batch dim
        t0 = time.time()

        # Inference
        prediction = self.model(torch_image, augment=self.augment, visualize=False)
        t1 = time.time()

        # NMS
        prediction = non_max_suppression(
            prediction,
            self.confidence_threshold,
            self.nms_iou_threshold,
            self.classes_filter,
            self.agnostic_nms,
            max_det=self.max_detections
        )
        t2 = time.time()

        detections = {}
        overlay_image = None

        assert len(prediction) <= 1
        if len(prediction) == 0:
            return detections, overlay_image

        detection = prediction[0]

        # Rescale boxes from torch_image size to image size
        detection[:, :4] = scale_coords(torch_image.shape[2:], detection[:, :4], image.shape).round()
        t3 = time.time()

        if self.publish_overlay:
            annotator = Annotator(np.copy(image), line_width=self.overlay_line_thickness, example=str(self.class_names))
            for *xyxy, confidence, class_index in reversed(detection):
                class_index = int(class_index)
                label = f"{self.class_names[class_index]} {confidence:.2f}"
                annotator.box_label(xyxy, label, color=colors(class_index, True))
            overlay_image = annotator.result()
        else:
            overlay_image = None
        t4 = time.time()

        class_count = {}
        for *xyxy, confidence, class_index in reversed(detection):
            if class_index not in class_count:
                class_count[class_index] = 0
            else:
                class_count[class_index] += 1

            obj_id = self.get_obj_id(class_index, class_count[class_index])
            bndbox = torch.tensor(xyxy).view(1, 4).view(-1).tolist()

            detections[obj_id] = bndbox, confidence
        t5 = time.time()

        if self.report_loop_times:
            self.timing_report = ""
            self.timing_report += "\ttensor prep: %0.4f\n" % (t0 - t_start)
            self.timing_report += "\tpredict: %0.4f\n" % (t1 - t0)
            self.timing_report += "\tnms: %0.4f\n" % (t2 - t1)
            self.timing_report += "\tscale: %0.4f\n" % (t3 - t2)
            self.timing_report += "\toverlay: %0.4f\n" % (t4 - t3)
            self.timing_report += "\tmsg: %0.4f\n" % (t5 - t4)

        return detections, overlay_image

    def get_obj_id(self, class_index, class_count):
        return get_obj_id(class_index, class_count)

    def get_label(self, obj_id):
        return get_label(self.class_names, obj_id)
