import onnx_graphsurgeon as gs
import onnx
import numpy as np

graph = gs.import_onnx(onnx.load("exported-models/dodobot_objects_ssd_resnet50_v1_fpn/model.onnx"))
print(graph.inputs)
for inp in graph.inputs:
    inp.dtype = np.float32

onnx.save(gs.export_onnx(graph), "exported-models/dodobot_objects_ssd_resnet50_v1_fpn/model.onnx")
