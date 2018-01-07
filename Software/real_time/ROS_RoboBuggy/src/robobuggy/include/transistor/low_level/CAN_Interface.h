#ifndef _TRANSISTOR_CAN_INTERFACE_H_
#define _TRANSISTOR_CAN_INTERFACE_H_

#include <ros/ros.h>
#include <socketcan_interface/socketcan.h>

#include <robobuggy/Feedback.h>
#include <robobuggy/Diagnostics.h>
#include <robobuggy/Encoder.h>

class CAN_Interface
{
public:
    CAN_Interface();
    static const std::string NODE_NAME;

    int initialize_CAN(); 
    void handle_CAN_messages();

private:
    ros::NodeHandle nh;

    ros::Publisher feedback_pub;
    ros::Publisher diagnostics_pub;
    ros::Publisher encoder_pub;

    int CAN_descriptor;
};

#endif /* _TRANSISTOR_CAN_INTERFACE_H_ */
