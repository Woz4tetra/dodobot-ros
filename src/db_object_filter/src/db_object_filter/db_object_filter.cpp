#include <db_object_filter/db_object_filter.h>


DodobotObjectFilter::DodobotObjectFilter(ros::NodeHandle* nodehandle) :
    nh(*nodehandle)
{
    ros::param::param<string>("~class_label_param", _class_label_param, "class_labels");
    string key;
    while (!ros::param::search(_class_label_param, key)) {
        if (!ros::ok()) {
            ROS_INFO_STREAM("db_object_filter interrupted while looking for " << _class_label_param);
            return;
        }
        ros::Duration(0.05).sleep();
    }

    ROS_INFO_STREAM("db_object_filter found the class list parameter: " << key);
    nh.getParam(key, _class_labels);

    _detection_sub = nh.subscribe<vision_msgs::Detection2DArray>("detections", 25, &DodobotObjectFilter::detections_callback, this);
    ROS_INFO("db_object_filter init done");
}

string DodobotObjectFilter::to_label(int index) {
    return _class_labels[index];
}

void DodobotObjectFilter::detections_callback(vision_msgs::Detection2DArray msg)
{
    for (size_t index = 0; index < msg.detections.size(); index++) {
        vision_msgs::ObjectHypothesisWithPose obj = msg.detections[index].results[0];
        ROS_INFO_STREAM("Found object: " << to_label(obj.id));
    }
}

int DodobotObjectFilter::run()
{
    ros::spin();

    return 0;
}
