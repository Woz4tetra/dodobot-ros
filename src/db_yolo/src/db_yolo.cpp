#include "db_yolo.h"

DodobotYolo::DodobotYolo(ros::NodeHandle* nodehandle) :
    nh(*nodehandle),
    _image_transport(nh),
    tfListener(tfBuffer)
{
    ros::param::param<std::string>("~model_path", _model_path, "best.pt");
    ros::param::param<std::string>("~classes_path", _classes_path, "coco.names");
    ros::param::param<std::string>("~model_path", _model_path, "best.pt");
    ros::param::param<float>("~confidence_threshold", _conf_threshold, 0.25);
    ros::param::param<float>("~nms_iou_threshold", _iou_threshold, 0.45);

    ros::param::param<std::string>("~image_width_param", _image_width_param, "/camera/realsense2_camera/color_width");
    ros::param::param<std::string>("~image_height_param", _image_height_param, "/camera/realsense2_camera/color_height");
    ros::param::param<int>(_image_width_param, _image_width, 960);
    ros::param::param<int>(_image_height_param, _image_height, 540);

    ros::param::param<int>("~circle_mask_border_offset_px", _circle_mask_border_offset_px, 0);
    ros::param::param<double>("~circle_mask_border_divisor", _circle_mask_border_divisor, 1.0);

    ros::param::param<bool>("~publish_overlay", _publish_overlay, true);
    ros::param::param<bool>("~report_loop_times", _report_loop_times, true);
    ros::param::param<std::string>("~target_frame", _target_frame, "base_link");
    ros::param::param<double>("~marker_persistance_s", _marker_persistance_s, 0.5);
    _marker_persistance = ros::Duration(_marker_persistance_s);

    std::string key;
    if (!ros::param::search("yolo_z_depth_estimations", key)) {
        ROS_WARN("Failed to find yolo_z_depth_estimations parameter");
    }
    ROS_DEBUG("Found yolo_z_depth_estimations: %s", key.c_str());
    nh.getParam(key, _z_depth_estimations);
    if (_z_depth_estimations.size() == 0) {
        ROS_WARN("yolo_z_depth_estimations has zero length!");
    }


    torch::DeviceType device_type;
    if (torch::cuda::is_available()) {
        device_type = torch::kCUDA;
    } else {
        device_type = torch::kCPU;
    }
    _class_names = Detector::LoadNames(_classes_path);
    if (_class_names.empty()) {
        std::cerr << "Error loading class names!\n";
        std::exit(EXIT_FAILURE);
    }
    _obj_count.resize(_class_names.size());

    _detector = new Detector(_model_path, device_type, _report_loop_times);
    // run once to warm up
    ROS_INFO("Warming up detector with (%dx%d)", _image_width, _image_height);
    auto temp_img = cv::Mat::zeros(_image_height, _image_width, CV_8UC3);
    _detector->Run(temp_img, _conf_threshold, _iou_threshold);

    // Subscribers
    _color_sub.subscribe(nh, "color/image_raw", 1);
    _depth_sub.subscribe(nh, "depth/image_raw", 1);

    _sync.reset(new Sync(ApproxSyncPolicy(10), _color_sub, _depth_sub));
    // _sync.reset(new Sync(ExactSyncPolicy(10), _color_sub, _depth_sub));
    _sync->registerCallback(boost::bind(&DodobotYolo::rgbd_callback, this, _1, _2));

    _color_info_sub = nh.subscribe<sensor_msgs::CameraInfo>("color/camera_info", 1, &DodobotYolo::camera_info_callback, this);

    _detection_pub = nh.advertise<vision_msgs::Detection3DArray>("detections", 25);
    _marker_pub = nh.advertise<visualization_msgs::MarkerArray>("markers", 25);

    _overlay_pub = _image_transport.advertise("overlay/image_raw", 2);
    _overlay_info_pub = nh.advertise<sensor_msgs::CameraInfo>("overlay/camera_info", 2);

    _dyn_cfg_wrapped_callback = boost::bind(&DodobotYolo::dynamic_callback, this, _1, _2);
    _dyn_cfg.setCallback(_dyn_cfg_wrapped_callback);

    ROS_INFO("db_yolo is ready");
}

void DodobotYolo::dynamic_callback(db_yolo::YoloDetectionConfig &config, uint32_t level)
{
    _conf_threshold = config.confidence_threshold;
    _iou_threshold = config.nms_iou_threshold;
    _publish_overlay = config.publish_overlay;
    _report_loop_times = config.report_loop_times;

    ROS_INFO("Setting config to:\n\tconfidence_threshold: %0.2f\n\tnms_iou_threshold: %0.2f\n\tpublish_overlay: %d\n\treport_loop_times: %d",
        _conf_threshold,
        _iou_threshold,
        _publish_overlay,
        _report_loop_times
    );
}

void DodobotYolo::camera_info_callback(const sensor_msgs::CameraInfoConstPtr& camera_info)
{
    _camera_info = *camera_info;
    _camera_model.fromCameraInfo(camera_info);
    _color_info_sub.shutdown();
    ROS_INFO("Camera model loaded");
}

void DodobotYolo::rgbd_callback(const sensor_msgs::ImageConstPtr& color_image, const sensor_msgs::ImageConstPtr& depth_image)
{
    ros::Time now = ros::Time::now();
    ros::Duration color_transport_delay = now - color_image->header.stamp;
    double color_transport_delay_ms = color_transport_delay.toSec() * 1000.0;
    ros::Duration depth_transport_delay = now - depth_image->header.stamp;
    double depth_transport_delay_ms = depth_transport_delay.toSec() * 1000.0;
    if (color_transport_delay_ms >= 100.0) {
        ROS_WARN_THROTTLE(0.5, "Color image has a large delay: %0.4f ms", color_transport_delay_ms);
    }
    if (depth_transport_delay_ms >= 100.0) {
        ROS_WARN_THROTTLE(0.5, "Depth image has a large delay: %0.4f ms", depth_transport_delay_ms);
    }

    cv_bridge::CvImagePtr color_ptr;
    try {
        color_ptr = cv_bridge::toCvCopy(color_image, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Failed to convert color image: %s", e.what());
        return;
    }

    cv_bridge::CvImagePtr depth_ptr;
    try {
        depth_ptr = cv_bridge::toCvCopy(depth_image);  // encoding: passthrough
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Failed to convert depth image: %s", e.what());
        return;
    }

    cv::Mat color_cv_image = color_ptr->image;
    cv::Mat depth_cv_image = depth_ptr->image;

    if (color_cv_image.cols == 0 || color_cv_image.rows == 0) {
        ROS_ERROR("Color image has a zero width dimension!");
        return;
    }
    if (depth_cv_image.cols == 0 || depth_cv_image.rows == 0) {
        ROS_ERROR("Depth image has a zero width dimension!");
        return;
    }

    auto t_start = std::chrono::high_resolution_clock::now();
    auto result = _detector->Run(color_cv_image, _conf_threshold, _iou_threshold);
    if (_report_loop_times) {
        ROS_INFO_THROTTLE(0.5, "----- %lu detections -----\nColor delay: %0.4f ms\nDepth delay: %0.4f ms\n%s", 
            result.size(),
            color_transport_delay_ms,
            depth_transport_delay_ms,
            _detector->GetTimingReport().c_str()
        );
    }
    vision_msgs::Detection3DArray detection_3d_arr_msg;
    detection_3d_arr_msg.header = color_image->header;

    if (result.empty()) {
        _detection_pub.publish(detection_3d_arr_msg);
        return;
    }
    auto t0 = std::chrono::high_resolution_clock::now();
    auto detection_time_s = std::chrono::duration<double>(t0 - t_start);
    if (detection_time_s.count() > 1.0) {
        ROS_INFO("Detector is warming up");
        return;
    }

    if (_publish_overlay && _overlay_pub.getNumSubscribers() > 0)
    {
        cv::Mat overlay = color_cv_image.clone();
        draw_overlay(overlay, result);
        sensor_msgs::ImagePtr overlay_msg = cv_bridge::CvImage(color_image->header, sensor_msgs::image_encodings::BGR8, overlay).toImageMsg();

        _overlay_pub.publish(overlay_msg);
        _overlay_info_pub.publish(_camera_info);
    }
    auto t1 = std::chrono::high_resolution_clock::now();

    vision_msgs::Detection2DArray detection_2d_arr_msg = detections_to_msg(result);
    detection_2d_arr_msg.header = color_image->header;
;
    visualization_msgs::MarkerArray marker_array;

    for (size_t index = 0; index < detection_2d_arr_msg.detections.size(); index++) {
        vision_msgs::Detection2D detection_2d_msg = detection_2d_arr_msg.detections[index];
        detection_2d_msg.header = color_image->header;
        std_msgs::ColorRGBA obj_color = get_detection_color(color_cv_image, detection_2d_msg);
        double z_dist = get_depth_from_detection(depth_cv_image, detection_2d_msg);
        vision_msgs::Detection3D detection_3d_msg = detection_2d_to_3d(detection_2d_msg, z_dist);
        tf_detection_pose_to_robot(detection_3d_msg);
        detection_3d_arr_msg.detections.push_back(detection_3d_msg);
        add_detection_to_marker_array(marker_array, detection_3d_msg, obj_color);
    }
    _detection_pub.publish(detection_3d_arr_msg);
    _marker_pub.publish(marker_array);
    auto t_end = std::chrono::high_resolution_clock::now();

    auto total_time = std::chrono::duration_cast<std::chrono::milliseconds>(t_end - t_start);
    if (total_time.count() >= 75) {
        ROS_WARN_THROTTLE(0.5, "Detection loop took a long time: %ld", total_time.count());
    }
    if (_report_loop_times) {
        auto detection_time = std::chrono::duration_cast<std::chrono::milliseconds>(t0 - t_start);
        auto overlay_time = std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0);
        auto message_prep_time = std::chrono::duration_cast<std::chrono::milliseconds>(t_end - t1);
        boost::format fmt = boost::format("\tdetection: %d ms\n\toverlay: %d ms\n\tmessage: %d ms\n\ttotal: %d ms\n") %
            detection_time.count() %
            overlay_time.count() %
            message_prep_time.count() %
            total_time.count();
        ROS_INFO_THROTTLE(0.5, "Loop report:\n%s", fmt.str().c_str());
    }
}

std_msgs::ColorRGBA DodobotYolo::get_detection_color(cv::Mat color_cv_image, vision_msgs::Detection2D detection_2d_msg)
{
    int center_x = (int)(detection_2d_msg.bbox.center.x);
    int center_y = (int)(detection_2d_msg.bbox.center.y);
    cv::Vec3b bgr_pixel = color_cv_image.at<cv::Vec3b>(center_y, center_x);
    double b = bgr_pixel[0] / 255.0;
    double g = bgr_pixel[1] / 255.0;
    double r = bgr_pixel[2] / 255.0;
    std_msgs::ColorRGBA color;
    color.r = r;
    color.g = g;
    color.b = b;
    color.a = 1.0;
    return color;
}

double DodobotYolo::get_depth_from_detection(cv::Mat depth_cv_image, vision_msgs::Detection2D detection_2d_msg)
{
    cv::Mat circle_mask = cv::Mat::zeros(depth_cv_image.rows, depth_cv_image.cols, CV_8UC1);
    int center_x = (int)(detection_2d_msg.bbox.center.x);
    int center_y = (int)(detection_2d_msg.bbox.center.y);
    double radius = (std::min(detection_2d_msg.bbox.size_x, detection_2d_msg.bbox.size_y) / 2.0);

    radius -= (double)_circle_mask_border_offset_px;
    if (_circle_mask_border_divisor > 0.0) {
        radius /= _circle_mask_border_divisor;    
    }
    if (radius < 1.0) {
        radius = 1.0;
    }
    int mask_radius = (int)radius;

    cv::circle(circle_mask, cv::Size(center_x, center_y), mask_radius, cv::Scalar(255, 255, 255), cv::FILLED);

    cv::Mat nonzero_mask = (depth_cv_image > 0.0);
    nonzero_mask.convertTo(nonzero_mask, CV_8U);

    cv::Mat target_mask;
    cv::bitwise_and(circle_mask, nonzero_mask, target_mask);

    double z_dist = cv::mean(depth_cv_image, target_mask)[0];  // depth values are in mm

    z_dist /= 1000.0;  // convert to meters

    return z_dist;
}

vision_msgs::Detection3D DodobotYolo::detection_2d_to_3d(vision_msgs::Detection2D detection_2d_msg, double z_dist)
{
    int center_x = (int)(detection_2d_msg.bbox.center.x);
    int center_y = (int)(detection_2d_msg.bbox.center.y);

    int size_x = (int)(detection_2d_msg.bbox.size_x);
    int size_y = (int)(detection_2d_msg.bbox.size_y);

    double z_model = 0.01;
    if (_z_depth_estimations.size() >= _class_names.size()) {
        z_model = _z_depth_estimations[get_class_name(detection_2d_msg.results[0].id)];
    }

    cv::Point2d center_point;
    center_point.x = center_x;
    center_point.y = center_y;
    cv::Point3d ray = _camera_model.projectPixelTo3dRay(center_point);

    double x_dist = ray.x * z_dist;
    double y_dist = ray.y * z_dist;

    cv::Point2d edge_point;
    edge_point.x = center_x - (int)(size_x / 2.0);
    edge_point.y = center_y - (int)(size_y / 2.0);
    ray = _camera_model.projectPixelTo3dRay(edge_point);

    double x_edge = std::abs(ray.x * z_dist - x_dist) * 2.0;
    double y_edge = std::abs(ray.y * z_dist - y_dist) * 2.0;

    geometry_msgs::Pose pose;

    pose.position.x = x_dist;
    pose.position.y = y_dist;
    pose.position.z = z_dist;
    pose.orientation.x = 0.0;
    pose.orientation.y = 0.0;
    pose.orientation.z = 0.0;
    pose.orientation.w = 1.0;

    vision_msgs::BoundingBox3D bbox;

    bbox.center = pose;
    bbox.size.x = x_edge;
    bbox.size.y = y_edge;
    bbox.size.z = z_model;

    vision_msgs::Detection3D detection_3d_msg;
    detection_3d_msg.header = detection_2d_msg.header;
    detection_3d_msg.results = detection_2d_msg.results;

    detection_3d_msg.results[0].pose.pose = bbox.center;
    detection_3d_msg.bbox = bbox;

    return detection_3d_msg;
}

void DodobotYolo::tf_detection_pose_to_robot(vision_msgs::Detection3D& detection_3d_msg)
{
    geometry_msgs::TransformStamped transform_camera_to_target;

    try {
        transform_camera_to_target = tfBuffer.lookupTransform(
            _target_frame, detection_3d_msg.header.frame_id,
            detection_3d_msg.header.stamp, ros::Duration(1.0)
        );
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header = detection_3d_msg.header;
        pose_stamped.pose = detection_3d_msg.bbox.center;
        tf2::doTransform(pose_stamped, pose_stamped, transform_camera_to_target);
        // pose_stamped now contains the object position in the target frame
        detection_3d_msg.header.frame_id = _target_frame;
        detection_3d_msg.bbox.center = pose_stamped.pose;
        detection_3d_msg.results[0].pose.pose = pose_stamped.pose;
    }
    catch (tf2::TransformException &ex) {
        ROS_WARN_THROTTLE(0.5, "%s", ex.what());
        return;
    }
}

std::string DodobotYolo::get_class_name(int obj_id)
{
    return _class_names[get_class_index(obj_id)];
}

int DodobotYolo::get_class_index(int obj_id)
{
    return obj_id & 0xffff;
}

int DodobotYolo::get_class_count(int obj_id)
{
    return obj_id >> 16;
}

vision_msgs::Detection2DArray DodobotYolo::detections_to_msg(const std::vector<std::vector<Detection>>& detections)
{
    vision_msgs::Detection2DArray detection_2d_arr_msg;
    for (size_t index = 0; index < _obj_count.size(); index++) {
        _obj_count[index] = 0;
    }

    for (const auto& detection : detections[0]) {
        const auto& box = detection.bbox;
        float score = detection.score;
        int class_idx = detection.class_idx;
        if (class_idx >= _obj_count.size()) {
            ROS_ERROR("Class index %d is not a registered class!", class_idx);
            continue;
        }
        int class_count = _obj_count[class_idx];
        _obj_count[class_idx]++;

        int obj_id = (class_count << 16) | class_idx;

        vision_msgs::Detection2D detection_2d_msg;

        detection_2d_msg.bbox.size_x = (double)box.width;
        detection_2d_msg.bbox.size_y = (double)box.height;

        detection_2d_msg.bbox.center.x = (double)box.x + (double)(box.width) / 2.0;
        detection_2d_msg.bbox.center.y = (double)box.y + (double)(box.height) / 2.0;

        detection_2d_msg.bbox.center.theta = 0.0;

        vision_msgs::ObjectHypothesisWithPose hyp;

        hyp.id = obj_id;
        hyp.score = score;

        detection_2d_msg.results.push_back(hyp);
        detection_2d_arr_msg.detections.push_back(detection_2d_msg);
    }
    return detection_2d_arr_msg;
}

void DodobotYolo::draw_overlay(cv::Mat img, const std::vector<std::vector<Detection>>& detections, bool label)
{
    if (detections.empty()) {
        return;
    }
    for (const auto& detection : detections[0]) {
        const auto& box = detection.bbox;
        float score = detection.score;
        int class_idx = detection.class_idx;

        cv::rectangle(img, box, cv::Scalar(0, 0, 255), 2);

        if (label) {
            std::stringstream ss;
            ss << std::fixed << std::setprecision(2) << score;
            std::string s = _class_names[class_idx] + " " + ss.str();

            auto font_face = cv::FONT_HERSHEY_DUPLEX;
            auto font_scale = 1.0;
            int thickness = 1;
            int baseline=0;
            auto s_size = cv::getTextSize(s, font_face, font_scale, thickness, &baseline);
            cv::rectangle(img,
                    cv::Point(box.tl().x, box.tl().y - s_size.height - 5),
                    cv::Point(box.tl().x + s_size.width, box.tl().y),
                    cv::Scalar(0, 0, 255), -1);
            cv::putText(img, s, cv::Point(box.tl().x, box.tl().y - 5),
                        font_face , font_scale, cv::Scalar(255, 255, 255), thickness);
        }
    }
}

void DodobotYolo::add_detection_to_marker_array(visualization_msgs::MarkerArray& marker_array, vision_msgs::Detection3D detection_3d_msg, std_msgs::ColorRGBA color)
{
    visualization_msgs::Marker sphere_marker = make_marker(detection_3d_msg, color);
    visualization_msgs::Marker text_marker = make_marker(detection_3d_msg, color);

    std::string label = get_class_name(detection_3d_msg.results[0].id);
    int count = get_class_count(detection_3d_msg.results[0].id);

    sphere_marker.type = visualization_msgs::Marker::SPHERE;
    sphere_marker.ns = "sphere_" + sphere_marker.ns;

    text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    text_marker.ns = "text_" + sphere_marker.ns;
    boost::format fmt = boost::format("%s_%s|%0.1f") % label % count % (detection_3d_msg.results[0].score * 100.0);
    text_marker.text = fmt.str();
    text_marker.scale.z = std::min({text_marker.scale.x, text_marker.scale.y});
    text_marker.scale.x = 0.0;
    text_marker.scale.y = 0.0;

    marker_array.markers.push_back(sphere_marker);
    marker_array.markers.push_back(text_marker);
}

visualization_msgs::Marker DodobotYolo::make_marker(vision_msgs::Detection3D detection_3d_msg, std_msgs::ColorRGBA color)
{
    std::string label = get_class_name(detection_3d_msg.results[0].id);
    int count = get_class_count(detection_3d_msg.results[0].id);

    visualization_msgs::Marker marker;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose = detection_3d_msg.bbox.center;
    marker.header = detection_3d_msg.header;
    marker.lifetime = _marker_persistance;
    marker.ns = label;
    marker.id = count;

    marker.scale = detection_3d_msg.bbox.size;
    marker.color = color;

    return marker;
}

int DodobotYolo::run()
{
    ros::spin();

    return 0;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "db_yolo");
    ros::NodeHandle nh;
    DodobotYolo node(&nh);
    return node.run();
}
