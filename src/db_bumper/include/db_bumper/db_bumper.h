#ifndef _DODOBOT_BUMPER_H_

#include <exception>
#include <iostream>
#include <ctime>
#include <math.h>

#include "ros/ros.h"
#include "ros/console.h"
#include <sensor_msgs/LaserScan.h>

#include "db_parsing/DodobotBumper.h"


using namespace std;

#define point pair<double, double>

class DodobotBumper {
private:
    ros::NodeHandle nh;  // ROS node handle

    ros::Publisher laser_scan_pub;

    // launch parameters
    string bumper_scan_topic;
    string bumper_frame;
    unsigned int scan_count;
    double range_max;
    vector<double> left_bumper_x_points;
    vector<double> left_bumper_y_points;
    vector<double> right_bumper_x_points;
    vector<double> right_bumper_y_points;
    vector<double> left_occupied_scan;
    vector<double> right_occupied_scan;

    // Subscribers
    ros::Subscriber bumper_sub;

    // Sub callbacks
    void bumper_callback(db_parsing::DodobotBumper msg);

    // messages
    sensor_msgs::LaserScan scan;
    void init_scan_msg();
    double get_min_value(vector<double>* points);
    double get_max_value(vector<double>* points);

    point get_line_intersection(point l1_a, point l1_b, point l2_a, point l2_b);
    double to_points_vector(vector<double>* input_x, vector<double>* input_y, vector<point>* output);
    double get_scan_dist(point p, double max_range);
    void apply_scan(bool left, bool right);

public:
    DodobotBumper(ros::NodeHandle* nodehandle);
    int run();
};

#endif  // _DODOBOT_BUMPER_H_
