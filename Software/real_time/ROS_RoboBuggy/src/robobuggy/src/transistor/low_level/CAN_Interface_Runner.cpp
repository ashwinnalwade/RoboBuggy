#include "transistor/low_level/CAN_Interface.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, CAN_Interface::NODE_NAME);
    ros::NodeHandle nh;

    CAN_Interface can_interface;
  
    int err = can_interface.initialize_CAN();

    if (err != 0) {
        ROS_INFO("Error initializing CAN interface\n");
    } else {
        can_interface.handle_CAN_messages();
    }

    return err;
}
