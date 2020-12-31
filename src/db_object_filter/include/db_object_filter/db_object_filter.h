#ifndef _DODOBOT_OBJECT_FILTER_H_

#include <exception>
#include <iostream>
#include <ctime>
#include <math.h>

#include "ros/ros.h"
#include "ros/console.h"


using namespace std;

class DodobotObjectFilter {
private:
    ros::NodeHandle nh;  // ROS node handle

public:
    DodobotObjectFilter(ros::NodeHandle* nodehandle);
    int run();
};

#endif  // _DODOBOT_OBJECT_FILTER_H_
