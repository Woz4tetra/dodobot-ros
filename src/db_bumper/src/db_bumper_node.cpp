
#include "db_bumper/db_bumper.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "db_bumper");
    ros::NodeHandle nh;

    DodobotBumper broadcaster(&nh);
    int err = broadcaster.run();

    return err;
}
