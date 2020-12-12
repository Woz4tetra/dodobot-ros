
#include "db_detectnet/db_detectnet.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "db_detectnet");
    ros::NodeHandle nh;

    DodobotDetectNet broadcaster(&nh);
    int err = broadcaster.run();

    return err;
}
