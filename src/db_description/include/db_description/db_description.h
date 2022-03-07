#pragma once

#include "ros/ros.h"
#include "ros/console.h"
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>

using namespace std;

class DodobotDescription {
private:
    ros::NodeHandle nh;  // ROS node handle

    std::vector<std::string> _joint_names;

    // Publishers
    ros::Publisher joint_pub;

    // Subscribers
    vector<ros::Subscriber>* raw_joint_subs;

    // Messages
    sensor_msgs::JointState joints_msg;

    // Module callback
    void joint_callback(const std_msgs::Float64ConstPtr& msg, int joint_index);

    // Main loop methods
    void loop();
public:
    DodobotDescription(ros::NodeHandle* nodehandle);
    int run();
};
