
#include "dodobot_serial_bridge/dodobot_serial_bridge.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "dodobot_serial_bridge");
    ros::NodeHandle nh;

    DodobotSerialBridge broadcaster(&nh);
    int err = broadcaster.run();

    return err;
}
