#ifndef __MOTORHANDLER_H
#define __MOTORHANDLER_H

#include <ros/ros.h>
#include <motors/Motor.h>
#include <nero_msgs/GripperControl.h>

/// Listens to motor commands and handles them accordingly.
class MotorHandler
{
protected:
    ros::NodeHandle mNodeHandle;    /// ROS node handle
    ros::Subscriber mMotorSub;		/// Listens to gripper motor commands
	Motor mGripperMotor;			/// Motor for left wheel
    ControlMode cmode;      		/// Keeps track of what ControlMode the motor is in at the moment

public:
    /// Constructor
    MotorHandler() : mNodeHandle(""), mGripperMotor(MID_GRIPPER), cmode(CM_NONE) { }
    
    /// Destructor
    /** shut down node handle */
    ~MotorHandler()
    {
        mNodeHandle.shutdown();
    }

    /// Initialize node
    /** \param path path to shared_serial node */
    void init(char *path);

    void gripperCB(const nero_msgs::GripperControl &mc);
};    

#endif /* __MOTORHANDLER_H */

