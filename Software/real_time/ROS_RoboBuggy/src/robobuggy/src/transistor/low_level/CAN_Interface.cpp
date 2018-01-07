#include "transistor/low_level/CAN_Interface.h"

const std::string CAN_Interface::NODE_NAME = "CAN_Interface";

CAN_Interface::CAN_Interface() {
    // Register publishers
    feedback_pub = nh.advertise<robobuggy::Feedback>("Feedback", 1000);
    diagnostics_pub = nh.advertise<robobuggy::Diagnostics>("Diagnostics", 1000);
    encoder_pub = nh.advertise<robobuggy::Encoder>("Encoder", 1000);
}

int CAN_Interface::initialize_CAN() {
    // Initialize the CAN bus
    struct sockaddr_can addr;
    struct ifreq ifr;

    const char *ifname = "can0"; //@TODO: Make this a parameter

    if ((CAN_descriptor = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
        ROS_ERROR("Error while opening socket\n");
        return -1;
    }

    strcpy(ifr.ifr_name, ifname);
    ioctl(CAN_descriptor, SIOCGIFINDEX, &ifr);

    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    ROS_INFO("%s at index %d\n", ifname, ifr.ifr_ifindex);

    if (bind(CAN_descriptor, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        ROS_ERROR("Error in socket bind\n");
        return -1;
    }

    ROS_INFO("Successfully initialized CAN interface\n");

    return 0;
}

void CAN_Interface::handle_CAN_messages() {
    // Assume that the CAN interface has been successfully initialized
    
    struct can_frame frame;

    while(ros::ok()) {
        // Read a CAN message
        int numRead = read(CAN_descriptor, &frame, sizeof(struct can_frame));

        ROS_INFO("Read CAN message with ID %d\n", frame.can_id);

        ros::spinOnce();
    }
}
