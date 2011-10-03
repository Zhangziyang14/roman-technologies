#ifndef __MOTORHANDLER_H
#define __MOTORHANDLER_H

#include <ros/ros.h>
#include <Motor.h>
#include <threemxl/example.h>
#include <threemxl/C3mxlROS.h>
#include <geometry_msgs/Twist.h>

/// Listens to motor commands and handles them accordingly.
class MotorHandler
{
protected:
    ros::NodeHandle nh_;             /// ROS node handle
    ros::Subscriber twist_sub;
    Motor left_engine;               /// Motor for left wheel
    Motor right_engine;              /// Motor for right wheel
    //Motor arm_engine;                /// Motor for arm

public:
    /// Constructor
    MotorHandler() : nh_("~"), left_engine(106), right_engine(107) { }
    
    /// Destructor
    /** Delete motor interface, close serial port, and shut down node handle */
    ~MotorHandler()
    {     
        nh_.shutdown();
    }

    /// Initialize node
    void init(char *path);

    void moveCB(const geometry_msgs::Twist& msg);
};    

#endif /* __MOTORHANDLER_H */

