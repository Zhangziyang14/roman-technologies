#ifndef __BASECONTROLLER_H
#define __BASECONTROLLER_H

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <numeric>
#include <vector>
#include <iostream>

enum PS3Key
{
    PS3_NONE                = -1,
	PS3_LEFT_HORIZONTAL     = 0,
	PS3_LEFT_VERTICAL       = 1,
	PS3_RIGHT_HORIZONTAL    = 2,
	PS3_RIGHT_VERTICAL      = 3,
	PS3_L2                  = 8,
	PS3_R2                  = 9,
	PS3_L1                  = 10,
	PS3_R1                  = 11,
	PS3_T                   = 12,
	PS3_O                   = 13,
	PS3_X                   = 14,
};

class BaseController
{
protected:
    ros::NodeHandle mNodeHandle;    /// ROS node handle
    ros::Subscriber mKey_sub;       /// Key input subscriber, reads key input data
    ros::Publisher mTwist_pub;      /// Twist message publisher, publishes movement data for engines

public:
    /// Constructor
    BaseController() : mNodeHandle("") { }
    
    /// Destructor
    ~BaseController()
    {

        mNodeHandle.shutdown();
    }

    /// Initialize node
    void init();

    /// Listen to PS3 controller
    void keyCB(const sensor_msgs::Joy& msg);
};

#endif /* __CONTROLLER_H */
