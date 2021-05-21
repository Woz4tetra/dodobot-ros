#!/bin/bash
# python3 exporter_main_v2.py --input_type image_tensor --pipeline_config_path ./models/my_faster_rcnn_resnet152_v1/pipeline.config --trained_checkpoint_dir ./models/my_faster_rcnn_resnet152_v1 --output_directory ./exported-models/dodobot_objects_faster_rcnn_resnet152_v1
# python3 exporter_main_v2.py --input_type image_tensor --pipeline_config_path ./pre-trained-models/faster_rcnn_resnet152_v1_1024x1024_coco17_tpu-8/pipeline.config --trained_checkpoint_dir ./pre-trained-models/faster_rcnn_resnet152_v1_1024x1024_coco17_tpu-8/checkpoint --output_directory ./exported-models/faster_rcnn_resnet152_v1
python3 exporter_main_v2.py --input_type image_tensor --pipeline_config_path ./models/my_ssd_mobilenet_v2/pipeline.config --trained_checkpoint_dir ./models/my_ssd_mobilenet_v2 --output_directory ./exported-models/dodobot_objects_ssd_mobilenet_v2

