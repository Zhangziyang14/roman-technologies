#ifndef __CONTROLLERTEST_H
#define __CONTROLLERTEST_H

#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>
#include <exception>
#include <gtest/gtest.h>
#include <cstdio>
#include "Controller.h"

///Tests Controller
class Controllertest
{
protected:
    ros::NodeHandle nh;   // ROS node handle
public:

    ros::Publisher keyErrorTest_pub;
    ros::Publisher keyTest_pub;

    ros::Subscriber sensorTest_sub;
    bool called;

    /// Constructor
    Controllertest() : nh(""){ }
    
    /// Destructor
    ~Controllertest()
    {
        nh.shutdown();
    }

    /// Initialize node
    void init();

    void activateSensorCBTest(const std_msgs::Empty::ConstPtr& msg);
};    

#endif /* __CONTROLLERTEST_H */
