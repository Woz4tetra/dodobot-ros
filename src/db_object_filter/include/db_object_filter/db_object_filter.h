#ifndef _DODOBOT_OBJECT_FILTER_H_

#include <exception>
#include <iostream>
#include <ctime>
#include <math.h>

#include "ros/ros.h"
#include "ros/console.h"

#include <pdf/conditionalpdf.h>
#include <pdf/gaussian.h>
// #include "nonlinearMeasurementPdf.h"
#include <wrappers/rng/rng.h>
#include <filter/bootstrapfilter.h>
#include <model/systemmodel.h>
#include <model/measurementmodel.h>

#include <vision_msgs/Detection2DArray.h>


using namespace std;

class DodobotObjectFilter {
private:
    ros::NodeHandle nh;  // ROS node handle

public:
    DodobotObjectFilter(ros::NodeHandle* nodehandle);

    // Launch parameters
    string _class_label_param;
    vector<string> _class_labels;

    // Class labels
    string to_label(int index);

    // Subscribers
    ros::Subscriber _detection_sub;

    // Callbacks
    void detections_callback(vision_msgs::Detection2DArray msg);

    int run();
};

#endif  // _DODOBOT_OBJECT_FILTER_H_
