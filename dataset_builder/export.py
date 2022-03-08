# https://docs.ultralytics.com/tutorials/torchscript-onnx-coreml-export/
import os

from yolov5 import export

export.run(
    data=os.path.abspath("outputs/dodobot_objects.yaml"),
    weights=os.path.abspath("outputs/dodobot_objects_train/exp2/weights/best.pt"),
    device=0,
    imgsz=(640, 640),
)
