#ifndef __MOTOR_H
#define __MOTOR_H

#include <CDxlGeneric.h>
#include <threemxl/C3mxlROS.h>
#include <string>

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
class Motor
{
protected:
    int engine_number;      /// Motor id (left, right or arm motor)
    CDxlGeneric *motor_;    /// Motor interface
    LxSerial serial_port_;  /// Serial port interface
    ControlMode cmode;      /// Keeps track of what ControlMode the motor is in at the moment

public:
    /// Constructor
    Motor(int id) : engine_number(id), motor_(NULL), cmode(CM_NONE) { }
    
    /// Destructor
    /** Delete motor interface, close serial port, and shut down node handle */
    ~Motor()
    {
        if (motor_)
            delete motor_;
        if (serial_port_.is_port_open())
            serial_port_.port_close();
          
    }

    /// Initialize node
    /** \param path path to shared_serial node */
    void init(char *path);

    void setMode(int mode);
    void setSpeed(double speed);
    void setAcceleration(double aceleration);
    void setPID(double p, double i, double d);
    void printPID();

    int getMode();
    int getID();
};    

#endif /* __MOTOR_H */
