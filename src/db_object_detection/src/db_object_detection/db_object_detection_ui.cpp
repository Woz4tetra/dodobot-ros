
#include <db_object_detection/db_object_detection_ui.h>


DodobotObjectDetectionUI::DodobotObjectDetectionUI(ros::NodeHandle* nodehandle):nh(*nodehandle),image_transport(nh)
{
    ros::param::param<string>("~img_topic", img_topic, "image_raw");

    image_sub = image_transport.subscribe(img_topic, 1, &DodobotObjectDetectionUI::imgCallback, this);

    cv_window_name = "Dodobot Object Detection";
    cv::namedWindow(cv_window_name, CV_WINDOW_AUTOSIZE);
    cv::setMouseCallback(cv_window_name, &DodobotObjectDetectionUI::onMouse, this);

    display_image = cv::Mat::zeros(cv::Size(300, 300), CV_8UC1);
    accept_image_msgs = true;
    is_selecting_object = false;
    selection_origin = cv::Point(0, 0);
    selection_box = cv::Rect(0, 0, 0, 0);

    ROS_INFO("db_object_detection_ui init done");
}

void DodobotObjectDetectionUI::imgCallback(const sensor_msgs::ImageConstPtr& msg)
{
    if (!accept_image_msgs) {
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
    display_image = cv_ptr->image;
}

void DodobotObjectDetectionUI::onMouse(int event, int x, int y, int, void* userdata)
{
    if (userdata == NULL) {
        ROS_ERROR("onMouse param is null!");
        return;
    }
    DodobotObjectDetectionUI* _this = reinterpret_cast<DodobotObjectDetectionUI*>(userdata);
    _this->onMouse(event, x, y);
}

void DodobotObjectDetectionUI::onMouse(int event, int x, int y)
{
    ROS_INFO("x: %d, y: %d", x, y);
    if (is_selecting_object) {
        selection_box.x = std::min(x, selection_origin.x);
        selection_box.y = std::min(y, selection_origin.y);
        selection_box.width = std::abs(x - selection_origin.x);
        selection_box.height = std::abs(y - selection_origin.y);
    }

    switch (event)
    {
        case cv::EVENT_LBUTTONDOWN:
            accept_image_msgs = false;
            is_selecting_object = true;
            selection_origin = cv::Point(x, y);
            selection_box = cv::Rect(x, y, 0, 0);
            break;
        case cv::EVENT_LBUTTONUP:
            accept_image_msgs = true;
            is_selecting_object = false;
            break;
    }
}


int DodobotObjectDetectionUI::run()
{
    ros::Rate clock_rate(30);  // run loop at 30 Hz
    while (ros::ok()) {
        ros::spinOnce();
        clock_rate.sleep();

        if (display_image.empty()) {
            continue;
        }

        cv::Mat drawn_image = display_image;
        cv::rectangle(drawn_image, selection_box, cv::Scalar(255, 255, 255), 2);
        cv::imshow(cv_window_name, drawn_image);
        char c = (char)cv::waitKey(1);
        if (c == 'q') {
            return 0;
        }

    }
    return 0;
}
