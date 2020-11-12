
#include "db_chassis/db_chassis.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "db_chassis");
    ros::NodeHandle nh;

    DodobotChassis broadcaster(&nh);
    int err = broadcaster.run();

    return err;
}
