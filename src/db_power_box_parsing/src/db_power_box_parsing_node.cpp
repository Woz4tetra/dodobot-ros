
#include <db_power_box_parsing/db_power_box_parsing.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "db_power_box_parsing");
    ros::NodeHandle nh;

    DodobotPowerBoxParsing broadcaster(&nh);
    int err = broadcaster.run();

    return err;
}
