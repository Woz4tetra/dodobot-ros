
#include "db_description/db_description.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "db_description");
    ros::NodeHandle nh;

    DodobotDescription broadcaster(&nh);
    int err = broadcaster.run();

    return err;
}
