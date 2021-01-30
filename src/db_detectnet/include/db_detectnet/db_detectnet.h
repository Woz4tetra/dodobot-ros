#ifndef _DODOBOT_DETECTNET_H_

#include <exception>
#include <iostream>
#include <ctime>
#include <math.h>

#include "ros/ros.h"
#include "ros/console.h"

#include <xmlrpcpp/XmlRpcValue.h>

#include <std_msgs/ColorRGBA.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TransformStamped.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

#include <vision_msgs/Detection2DArray.h>
#include <vision_msgs/BoundingBox2D.h>

#include <image_geometry/pinhole_camera_model.h>

#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/image_encodings.h>

#include <jetson-inference/detectNet.h>
#include <db_detectnet/image_converter.h>

using namespace std;
using namespace sensor_msgs;

#define THROW_EXCEPTION(msg)  throw std::runtime_error(msg)


typedef struct {
    string label;
    int index;
    int center_x;
    int center_y;
    int obj_radius;
    int detect_radius;

    double x_dist;
    double y_dist;
    double z_dist;

    double x_edge;
    double y_edge;
    double z_edge;

    geometry_msgs::PoseStamped pose_stamped;
} ObjPoseDescription;

ObjPoseDescription init_obj_desc()
{
    return {
        .label = "",
        .index = 0,
        .center_x = 0,
        .center_y = 0,
        .obj_radius = 0,
        .detect_radius = 0,

        .x_dist = 0.0,
        .y_dist = 0.0,
        .z_dist = 0.0,

        .x_edge = 0.0,
        .y_edge = 0.0,
        .z_edge = 0.0,

        .pose_stamped = geometry_msgs::PoseStamped()
    };
}

class DodobotDetectNet {
private:
    ros::NodeHandle nh;  // ROS node handle

    // launch parameters
    string _model_name;
    string _model_path;
    string _prototxt_path;
    string _class_labels_path;

    string _input_blob;
    string _output_cvg;
    string _output_bbox;
    uint32_t _overlay_flags;
    string _overlay_str;

    double _mean_pixel;
    double _threshold;

    string _color_topic;
    string _color_info_topic;
    string _depth_topic;
    string _depth_info_topic;
    int _bounding_box_border_px;
    double _marker_persistance_s;
    ros::Duration _marker_persistance;

    bool _publish_with_frame;
    string _target_frame;

    XmlRpc::XmlRpcValue _marker_colors_param;
    std::map<std::string, std_msgs::ColorRGBA> _marker_colors;
    std::map<std::string, double> _z_depth_estimations;

    std::vector<std::string> _class_descriptions;
    uint32_t _num_classes;

    // detectnet variables
    detectNet* _net;

    imageConverter* _input_cvt;
    imageConverter* _overlay_cvt;

    void load_detectnet_model();
    void load_labels();

    // bbox to pose variables
    image_geometry::PinholeCameraModel _camera_model;
    std::map<std::string, int> _label_counter;


    // Subscribers
    message_filters::Subscriber<Image> color_sub;
    message_filters::Subscriber<CameraInfo> color_info_sub;
    message_filters::Subscriber<Image> depth_sub;

    typedef message_filters::sync_policies::ApproximateTime<Image, CameraInfo, Image> ApproxSyncPolicy;
    // typedef message_filters::sync_policies::ExactTime<Image, CameraInfo, Image> ExactSyncPolicy;

    typedef message_filters::Synchronizer<ApproxSyncPolicy> Sync;
    // typedef message_filters::Synchronizer<ExactSyncPolicy> Sync;
    boost::shared_ptr<Sync> sync;

    // Publishers
    image_transport::ImageTransport _image_transport;

    ros::Publisher _detection_pub;
    ros::Publisher _marker_pub;
    image_transport::Publisher _overlay_pub;

    // ROS TF
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener;

    // Sub callbacks
    void rgbd_callback(const ImageConstPtr& color_image, const CameraInfoConstPtr& color_info, const ImageConstPtr& depth_image);
    int detect(const ImageConstPtr& color_image, vision_msgs::Detection2DArray* msg);
    bool publish_overlay(detectNet::Detection* detections, int num_detections);

    void reset_label_counter();
    ObjPoseDescription bbox_to_pose(cv::Mat depth_cv_image, vision_msgs::BoundingBox2D bbox, ros::Time stamp, string label, int label_index);
    visualization_msgs::Marker make_marker(ObjPoseDescription* desc);
    double get_z_dist(cv::Mat depth_cv_image, ObjPoseDescription* desc);
    void tf_obj_to_target(const CameraInfoConstPtr color_info, ObjPoseDescription& obj_desc);

public:
    DodobotDetectNet(ros::NodeHandle* nodehandle);
    int run();
};

#endif  // _DODOBOT_DETECTNET_H_
