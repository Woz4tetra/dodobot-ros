#ifndef _DODOBOT_OBJECT_FILTER_H_

#include <exception>
#include <iostream>
#include <ctime>
#include <math.h>

#include "ros/ros.h"
#include "ros/console.h"

#include <xmlrpcpp/XmlRpcValue.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>

#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

#include <vision_msgs/Detection2DArray.h>
#include <vision_msgs/BoundingBox2D.h>


using namespace std;

class DodobotObjectFilter {
private:
    ros::NodeHandle nh;  // ROS node handle

    // launch parameters

    XmlRpc::XmlRpcValue _marker_colors_param;
    std::map<std::string, std_msgs::ColorRGBA> _marker_colors;
    std::map<std::string, double> _z_depth_estimations;

    std::vector<std::string> _class_descriptions;

    // Subscribers
    ros::Subscriber _detection_sub;
    ros::Subscriber _twist_sub;

    // Publishers
    ros::Publisher _detection_sub;

    // Sub callbacks

public:
    DodobotObjectFilter(ros::NodeHandle* nodehandle);
    int run();
};

#endif  // _DODOBOT_OBJECT_FILTER_H_
