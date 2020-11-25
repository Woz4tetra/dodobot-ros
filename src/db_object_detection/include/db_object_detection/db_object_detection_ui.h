#ifndef _DODOBOT_OBJECT_DETECTION_UI_H_

#include <exception>
#include <iostream>
#include <ctime>
#include <math.h>

#include "ros/ros.h"
#include "ros/console.h"

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include "opencv2/core/utility.hpp"
#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/highgui.hpp"

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include "sensor_msgs/Image.h"

using namespace std;


class DodobotObjectDetectionUI {
private:
    ros::NodeHandle nh;  // ROS node handle

    string img_topic;
    string cv_window_name;

    cv::Mat display_image;

    bool accept_image_msgs;
    bool is_selecting_object;

    cv::Point selection_origin;
    cv::Rect selection_box;

    image_transport::ImageTransport image_transport;

    // Subscribers
    image_transport::Subscriber image_sub;

    void imgCallback(const sensor_msgs::ImageConstPtr& msg);
    static void onMouse(int event, int x, int y, int, void* userdata);
    void onMouse(int event, int x, int y);

public:
    DodobotObjectDetectionUI(ros::NodeHandle* nodehandle);
    int run();
};

#endif  // _DODOBOT_OBJECT_DETECTION_UI_H_
