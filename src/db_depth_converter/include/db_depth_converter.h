#pragma once
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"

#include <cv_bridge/cv_bridge.h>

#include <image_transport/camera_publisher.h>
#include <image_transport/image_transport.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>

using namespace std;


class DodobotDepthConverter
{
public:
    DodobotDepthConverter(ros::NodeHandle* nodehandle);
    int run();
private:
    ros::NodeHandle nh;  // ROS node handle

    // Parameters
    double _connected_components_size_threshold;
    int _erosion_size;
    double _throttle_frame_rate;
    double _rescale;

    // Members
    image_transport::ImageTransport _image_transport;
    cv::Mat _erode_element;
    sensor_msgs::CameraInfo _depth_info;
    bool _depth_info_set;
    ros::Time _prev_frame_time;
    ros::Duration _prev_frame_delay;

    // Publishers
    image_transport::CameraPublisher _depth_filtered_pub;

    // Subscribers
    image_transport::Subscriber _depth_sub;
    ros::Subscriber _info_sub;

    // Sub callbacks
    void depthCallback(const sensor_msgs::ImageConstPtr& depth_image);
    void infoCallback(const sensor_msgs::CameraInfoConstPtr& depth_info);
};
