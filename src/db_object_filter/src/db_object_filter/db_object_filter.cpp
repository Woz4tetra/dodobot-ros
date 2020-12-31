#include <db_object_filter/db_object_filter.h>


DodobotObjectFilter::DodobotObjectFilter(ros::NodeHandle* nodehandle) :
    nh(*nodehandle)
{
    ROS_INFO("db_object_filter init done");
}


int DodobotObjectFilter::run()
{
    ros::spin();

    return 0;
}
