# https://docs.ultralytics.com/tutorials/torchscript-onnx-coreml-export/
import os
import shutil

from yolov5 import export

export.run(
    data=os.path.abspath("outputs/dodobot_objects.yaml"),
    weights=os.path.abspath("outputs/dodobot_objects_train/exp/weights/best.pt"),
    device=0,
    imgsz=(640, 640),
)

shutil.copy("resources/yolo-dataset/classes.txt", "outputs/dodobot_objects_train/dodobot_objects.names")
