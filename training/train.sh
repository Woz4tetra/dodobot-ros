#!/bin/bash
# python3 model_main_tf2.py --model_dir=models/my_ssd_resnet50_v1_fpn --pipeline_config_path=models/my_ssd_resnet50_v1_fpn/pipeline.config --checkpoint_every_n=100  num_steps_per_iteration=100 #--num_train_steps=5000
# python3 model_main_tf2.py --model_dir=models/my_faster_rcnn_resnet152_v1 --pipeline_config_path=models/my_faster_rcnn_resnet152_v1/pipeline.config
# python3 model_main_tf2.py --model_dir=models/my_ssd_mobilenet_v2 --pipeline_config_path=models/my_ssd_mobilenet_v2/pipeline.config
# python3 model_main_tf2.py --model_dir=models/my_centernet_mobilenetv2_fpn_od --pipeline_config_path=models/my_centernet_mobilenetv2_fpn_od/pipeline.config
python3 model_main_tf2.py --model_dir=models/my_efficientdet_d1 --pipeline_config_path=models/my_efficientdet_d1/pipeline.config

