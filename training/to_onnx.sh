python3 -m tf2onnx.convert --saved-model exported-models/dodobot_objects_ssd_resnet50_v1_fpn/saved_model/ --output exported-models/dodobot_objects_ssd_resnet50_v1_fpn/model.onnx --opset 13
python3 fix_onnx_model.py

# 13 -> attribute not found: axes
# 12 -> INVALID_ARGUMENT: getPluginCreator could not find plugin TensorListStack version 1
# 11 -> INVALID_ARGUMENT: getPluginCreator could not find plugin TensorListStack version 1
# 10 -> ValueError: get tensor value: 'StatefulPartitionedCall/Postprocessor/BatchMultiClassNonMaxSuppression/PadOrClipBoxList/stack_Concat__972' must be Const
# 9 -> ValueError: StridedSlice: attribute new_axis_mask not supported
# 8 -> ValueError: StridedSlice: attribute new_axis_mask not supported

