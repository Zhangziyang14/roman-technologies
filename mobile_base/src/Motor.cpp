#include <Motor.h>

/**
 * Square bracket operator so it can be used as a sort of array.
 */
double &PID::operator[](PIDParameter p)
{
	switch (p)
	{
	case PID_PARAM_P: return this->p;
	case PID_PARAM_I: return this->i;
	case PID_PARAM_D: return this->d;
	default: break;
	}

	assert(false); // something went terribad
}

/**
  * Sets the mode of the motor
 */
void Motor::setMode(ControlMode mode)
{
	if (cmode == mode)
		return; // already in this mode, nothing needs to be done

	cmode = mode;
	motor_->set3MxlMode(mode);
	std::cout << "Changed to ";
    switch (mode)
    {
    case SPEED_MODE: std::cout << "speed"; break;
    case TORQUE_MODE: std::cout << "torque"; break;
    case CURRENT_MODE: std::cout << "current"; break;
    case STOP_MODE: std::cout << "stop"; break;
    case POSITION_MODE: std::cout << "position"; break;
    case SEA_MODE: std::cout << "sea"; break;
    case PWM_MODE: std::cout << "pwm"; break;
    case TEST_MODE: std::cout << "test"; break;
    default : break;
    }
    std::cout << " mode." << std::endl;
}


/**
  * Sets acceleration of the motor when in SPEED_MODE
 */
void Motor::setAcceleration(double acceleration)
{
    motor_->setAcceleration(acceleration);
}

/**
  * Sets speed of the motor when in SPEED_MODE
 */
void Motor::setSpeed(double speed)
{
	if (cmode != CM_SPEED_MODE)
	{
        std::cout << "Motor not in speed mode, setting it now." << std::endl;
        setMode(CM_SPEED_MODE);

		setAcceleration(DEFAULT_ACCELERATION);
	}

    motor_->setSpeed(speed);
}

/**
  * Sets speed of the motor when in SPEED_MODE
 */
void Motor::setLSpeed(double speed)
{
	if (cmode != CM_SPEED_MODE)
	{
        std::cout << "Motor not in speed mode, setting it now." << std::endl;
        setMode(CM_SPEED_MODE);

		setAcceleration(DEFAULT_ACCELERATION);
	}

    motor_->setLinearSpeed(speed);
}

/**
  * Sets position of the motor when in POSITION_MODE
 */
void Motor::setPosition(double position)
{
	if (cmode != CM_POSITION_MODE)
	{
        std::cout << "Motor not in POSITON mode, setting it now." << std::endl;
        setMode(CM_POSITION_MODE);
	}

	motor_->setLinearPos(position, 1, 0.5, false);
}

/**
 * Returns position of motor in meters
 */
double Motor::getPosition()
{
	motor_->getLinearPos();
	return motor_->presentLinearPos()*100;
}

/**
  * Gets speed of the motor when in SPEED_MODE
 */
double Motor::getRotationSpeed()
{
	motor_->getState();

	//ROS_INFO("PRESENT SPEED, %f rad/s", motor_->presentSpeed());
    return motor_->presentSpeed();
}

/**
 * Sets speed pid
 */
void Motor::updatePID()
{
    motor_->setPIDSpeed(mPID.p, mPID.d, mPID.i, 0, false);
}

/**
 * Gets speed pid
 */
void Motor::printPID()
{
	double p, i, d, i_limit;
	motor_->getPIDSpeed(p, d, i, i_limit);

	ROS_INFO("p: %f, i: %f, d: %f, i_limit: %f \n", p, i, d, i_limit);
}

int Motor::getLog()
{
	motor_->getLog();
	//return motor_->presentLog();
	return 0;
}

/**
 * Pings the motor
 */
int Motor::ping()
{
	return motor_->ping();
}
/**
 * Initalize Motor and its attributes.
*/
void Motor::init(char *path)
{
    CDxlConfig *config = new CDxlConfig();

    // find the connection to the motor on the 3mXl board
    if (path)
    {
        std::cout << "Using shared_serial"<< std::endl;
        motor_ = new C3mxlROS(path);
    }
    else
    {
        std::cout << "Using direct connection" << std::endl;
        motor_ = new C3mxl();

        serial_port_.port_open("/dev/motorusb", LxSerial::RS485_FTDI);
        serial_port_.set_speed(LxSerial::S921600);
        motor_->setSerialPort(&serial_port_);    
    }

    // initialize the motor
    motor_->setConfig(config->setID(mMotorId));
    motor_->init(false);
    setMode(CM_STOP_MODE);
    motor_->setEncoderCountMotor(500); // hack until motor is flashed with correct number

    delete config;
    std::cout << "Motor " << mMotorId << " initialising completed." << std::endl;
}
