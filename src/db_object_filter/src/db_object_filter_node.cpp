
#include "db_object_filter/db_object_filter.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "db_object_filter");
    ros::NodeHandle nh;

    DodobotObjectFilter broadcaster(&nh);
    int err = broadcaster.run();

    return err;
}
