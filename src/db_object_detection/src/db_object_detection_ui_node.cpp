
#include "db_object_detection/db_object_detection_ui.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "db_object_detection_ui");
    ros::NodeHandle nh;

    DodobotObjectDetectionUI broadcaster(&nh);
    int err = broadcaster.run();

    return err;
}
