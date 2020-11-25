
#include "db_object_detection/db_object_detection.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "db_object_detection");
    ros::NodeHandle nh;

    DodobotObjectDetection broadcaster(&nh);
    int err = broadcaster.run();

    return err;
}
