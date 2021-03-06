/*
 * DynamixelMotor.h
 *
 *  Created on: 2012-01-13
 *      Author: wilson
 */

#ifndef DYNAMIXELMOTOR_H_
#define DYNAMIXELMOTOR_H_

#include <ros/ros.h>
#include <Dynamixel.h>
#include <threemxl/CDynamixelROS.h>

#define DYNAMIXEL_MOTOR_SPEED 0.5

enum MotorId
{
	MID_NONE 	= -1,
	PAN_ID 		= 104,
	TILT_ID,
};

class DynamixelMotor
{
protected:
    MotorId mMotorId;      	/// Motor id (left, right or arm motor)
    CDynamixel *motor_;    	/// Motor interface
    LxSerial serial_port_;  /// Serial port interface

public:
    /// Constructor
    DynamixelMotor(MotorId id) : mMotorId(id), motor_(NULL){ }

    /// Destructor
    /** Delete motor interface, close serial port, and shut down node handle */
    ~DynamixelMotor()
    {
        if (motor_)
            delete motor_;
        if (serial_port_.is_port_open())
            serial_port_.port_close();
    }

    /// Initialize node
    /** \param path path to shared_serial node */
    void init(char *path);
    int update();
    double getRotationSpeed();
    void setPosition(double position);
    void setSpeed(double speed);
    double getPosition();
    void setAngleLimits(double lower, double upper);
    double getAngleLowerLimit();
    double getAngleUpperLimit();
    int ping();
};

#endif /* DYNAMIXELMOTOR_H_ */
