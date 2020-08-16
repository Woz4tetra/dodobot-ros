
#include "db_parsing/db_parsing.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "db_parsing");
    ros::NodeHandle nh;

    DodobotParsing broadcaster(&nh);
    int err = broadcaster.run();

    return err;
}
