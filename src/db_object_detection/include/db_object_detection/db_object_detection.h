#ifndef _DODOBOT_OBJECT_DETECTION_H_

#include <exception>
#include <iostream>
#include <ctime>
#include <math.h>

#include "ros/ros.h"
#include "ros/console.h"

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include "sensor_msgs/Image.h"

using namespace std;

class DodobotObjectDetection {
private:
    ros::NodeHandle nh;  // ROS node handle

    string img_topic;

    image_transport::ImageTransport image_transport;

    // Subscribers
    image_transport::Subscriber image_sub;

    // Publishers
    image_transport::Publisher debug_image_pub;

    void imgCallback(const sensor_msgs::ImageConstPtr& msg);


public:
    DodobotObjectDetection(ros::NodeHandle* nodehandle);
    int run();
};

#endif  // _DODOBOT_OBJECT_DETECTION_H_
