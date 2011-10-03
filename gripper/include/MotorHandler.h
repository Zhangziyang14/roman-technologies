#ifndef __MOTORHANDLER_H
#define __MOTORHANDLER_H

#include <ros/ros.h>
#include <CDxlGeneric.h>
#include <MotorControl.h>

enum ControlMode
{
    CM_NONE = -1,
	CM_POSITION_MODE,	// uses one motor and encoder with position controller
	CM_SPEED_MODE,		// uses one motor and encoder with speed controller
	CM_CURRENT_MODE,	// uses one motor and encoder with current controller
	CM_TORQUE_MODE,		// uses one motor and encoder with torque controller
	CM_SEA_MODE,		// uses one motor and two encoders with SEA controller
	CM_PWM_MODE,		// uses one motor and encoder with PWM controller, no PID, just PWM !!

    CM_STOP_MODE = 12,	// do nothing, no actuated motors
    CM_TEST_MODE,       // for general testing
};

/// Listens to motor commands and handles them accordingly.
class MotorHandler
{
protected:
    ros::NodeHandle nh_;    /// ROS node handle
    CDxlGeneric *motor_;    /// Motor interface
    LxSerial serial_port_;  /// Serial port interface
    ControlMode cmode;      /// Keeps track of what ControlMode the motor is in at the moment

public:
    /// Constructor
    MotorHandler() : nh_("~"), motor_(NULL), cmode(CM_NONE) { }
    
    /// Destructor
    /** Delete motor interface, close serial port, and shut down node handle */
    ~MotorHandler()
    {
        if (motor_)
            delete motor_;
        if (serial_port_.is_port_open())
            serial_port_.port_close();
          
        nh_.shutdown();
    }

    /// Initialize node
    /** \param path path to shared_serial node */
    void init(char *path);

    void chatterCallback(const gripper::MotorControlPtr& mc);
};    

#endif /* __MOTORHANDLER_H */

