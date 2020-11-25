
#include <db_object_detection/db_object_detection.h>


DodobotObjectDetection::DodobotObjectDetection(ros::NodeHandle* nodehandle):nh(*nodehandle),image_transport(nh)
{
    ros::param::param<string>("~img_topic", img_topic, "image_raw");

    debug_image_pub = image_transport.advertise("debug_image", 1);
    image_sub = image_transport.subscribe(img_topic, 1, &DodobotObjectDetection::imgCallback, this);

    ROS_INFO("db_object_detection init done");
}

void DodobotObjectDetection::imgCallback(const sensor_msgs::ImageConstPtr& msg)
{
    if (debug_image_pub.getNumSubscribers() == 0) {
        return;
    }
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    cv::Mat frame;
    cv::cvtColor(cv_ptr->image, frame, cv::COLOR_BGR2HSV);

    cv_ptr->image = frame;
    // AdaptiveThreshold(img, image_final, 255,ADAPTIVE_THRESH_GAUSSIAN_C,CV_THRESH_BINARY,13,0);

    debug_image_pub.publish(cv_ptr->toImageMsg());
}

int DodobotObjectDetection::run()
{
    ros::spin();
    return 0;
}
