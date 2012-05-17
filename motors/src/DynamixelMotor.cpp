/*
 * DynamixelMotor.cpp
 *
 *  Created on: 2012-01-13
 *      Author: wilson
 */

#include <motors/DynamixelMotor.h>

/**
  * Sets speed of the motor when in SPEED_MODE
 */
void DynamixelMotor::setSpeed(double speed)
{
    motor_->setSpeed(speed);
}

/**
  * Gets speed of the motor
 */
double DynamixelMotor::getRotationSpeed()
{
	motor_->getState();

	//ROS_INFO("PRESENT SPEED, %f rad/s", motor_->presentSpeed());
    return motor_->presentSpeed();
}

/**
 *	Gets Status of Dynamixel motor
 */
int DynamixelMotor::update()
{
	return motor_->getState();
}

/**
 * Pings the Dynamixel
 */
int DynamixelMotor::ping()
{
	return motor_->ping();
}

/**
 * Sets Dynamixel motor to given position
 */
void DynamixelMotor::setPosition(double position)
{
	motor_->setPos(position, DYNAMIXEL_MOTOR_SPEED, false);
}

/**
 * Returns position of Dynamixel in rad
 */
double DynamixelMotor::getPosition()
{
	motor_->getPos();
	return motor_->presentPos();
}

/**
 *	Sets angle lower limit
 */
void DynamixelMotor::setAngleLimits(double lower, double upper)
{
	motor_->setAngleLimits(lower, upper);
}


/**
 *	Returns angle lower limit
 */
double DynamixelMotor::getAngleLowerLimit()
{
	motor_->getAngleLimits();

	return motor_->presentAngleLowerLimit();
}

/**
 *	Returns angle upper limit
 */
double DynamixelMotor::getAngleUpperLimit()
{
	motor_->getAngleLimits();

	return motor_->presentAngleUpperLimit();
}

/**
 * Initialise Dynamixel motor
 */
void DynamixelMotor::init(char *path)
{
	CDxlConfig *config = new CDxlConfig();

	// find the connection to the motor on the dynamixel
	motor_ = path ? new CDynamixelROS(path) : new CDynamixel();
	/*if (path)
	{
		std::cout << "Using shared_serial"<< std::endl;
		motor_ = new CDynamixelROS(path);
	}
	else
	{
		std::cout << "Using direct connection" << std::endl;
		motor_ = new CDynamixel();

		serial_port_.port_open("/dev/roman/dynamixel", LxSerial::RS485_FTDI);
		serial_port_.set_speed(LxSerial::S1000000);

		motor_->setSerialPort(&serial_port_);
	}*/
	motor_->setSerialPort(&serial_port_);

	ros::Rate init_rate(1);
	while (ros::ok() && motor_->init() != DXL_SUCCESS)
	{
		ROS_WARN_ONCE("Couldn't initialize motor, will continue trying every second");
		init_rate.sleep();
	}
	
	if (motor_->isInitialized() == false)
	{
		ROS_ERROR("Failed to initialize motor.");
		return;
	}
	
	if (path == NULL)
	{
		serial_port_.port_open("/dev/roman/dynamixel", LxSerial::RS485_FTDI);
		serial_port_.set_speed(LxSerial::S1000000);
	}
	else
		std::cout << "Using shared_serial" << std::endl;

	// initialize the motor
	motor_->setConfig(config->setID(mMotorId));
	motor_->init(false);

	delete config;
	std::cout << "DynamixelMotor " << mMotorId << " initialising completed." << std::endl;
}
