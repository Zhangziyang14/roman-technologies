#include <motors/Motor.h>

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

void Motor::lock(bool lock)
{
	mLock = lock;
}

/**
  * Sets the mode of the motor
 */
void Motor::setMode(ControlMode mode)
{
	if (cmode == mode || mLock)
		return; // already in this mode, nothing needs to be done

	cmode = mode;
	motor_->set3MxlMode(mode);
	std::cout << "Changed to ";
    switch (mode)
    {
    case SPEED_MODE: std::cout << "speed"; break;
    case TORQUE_MODE: std::cout << "torque"; break;
    case EXTERNAL_INIT: std::cout << "external init"; break;
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

void Motor::setEncoderCount(int resolution)
{
	ROS_INFO("Setting encoder count to [%i]", resolution);
	motor_->setEncoderCountMotor(resolution);
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
void Motor::setLSpeed(double speed, double acceleration)
{
	assertMode(CM_SPEED_MODE);
    motor_->setLinearSpeed(speed, acceleration);
}

/**
  * Sets speed of the motor when in SPEED_MODE
 */
void Motor::setSpeed(double speed)
{
	assertMode(CM_SPEED_MODE);
    motor_->setSpeed(speed);
}

/**
  * Stops the base when in POSITION_MODE
 */
void Motor::stopAtPosition(double current_position)
{
	assertMode(CM_POSITION_MODE);
	motor_->setLinearPos(current_position, 0, 0.1, false);
}

/**
  * Sets linear position of the motor when in POSITION_MODE
 */
void Motor::setPosition(double position)
{
	assertMode(CM_POSITION_MODE);
	motor_->setLinearPos(position, 1, 0.1, false);
}

/**
 * Sets current of the motor when in CURRENT_MODE
 */
void Motor::setCurrent(double current)
{
	assertMode(CM_CURRENT_MODE);
	motor_->setCurrent(current);
}

void Motor::setAngle(double angle)
{
	assertMode(CM_POSITION_MODE);
	DXL_FORCE_CALL(motor_->setPos(angle));	//TODO DEBUG do we need DXL_SAFE_CALL ?
}
void Motor::setAngle(double angle, double speed)
{
	assertMode(CM_POSITION_MODE);
	DXL_FORCE_CALL(motor_->setPos(angle, speed));
}
void Motor::setAngle(double angle, double speed, double accel)
{
	assertMode(CM_POSITION_MODE);
	DXL_FORCE_CALL(motor_->setPos(angle, speed, accel));
}
double Motor::getAngle()
{
	DXL_FORCE_CALL(motor_->getPos());
	return motor_->presentPos();
}


void Motor::setTorque(double torque)
{
	assertMode(CM_TORQUE_MODE);
	motor_->setTorque(torque);
}


// check if the motor is in the correct control mode.
// if not, the control mode is set and a warning is emitted
void Motor::assertMode(ControlMode mode)
{
	if ((cmode != mode) && (cmode != CM_EXT_INIT_MODE))
	{
		std::cout << "Motor not in specified mode [" << mode << "], setting it now." << std::endl;
		setMode(mode);
	}
}

int Motor::update()
{
	return motor_->getStatus();
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

/** Returns the motor status
 *
 * @return	int		status code of the motor.
 *
 * @see 3mxlControlTable.h
 */
int Motor::getStatus()
{
	this->update();
	return motor_->presentStatus();
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

        serial_port_.port_open(THREEMXL_SERIAL_DEVICE, LxSerial::RS485_FTDI);
        serial_port_.set_speed(LxSerial::S921600);
        motor_->setSerialPort(&serial_port_);
    }

    // initialize the motor
    motor_->setConfig(config->setID(mMotorId));
    mLock = false;

    /*ros::Rate init_rate(1);
    	while (ros::ok() && motor_->init() != DXL_SUCCESS)
    	{
    		ROS_WARN_ONCE("Couldn't initialize motor, will continue trying every second");
    		init_rate.sleep();
    	}
*/
    motor_->init(false);
    setMode(CM_STOP_MODE);
    setEncoderCount(DEFAULT_ENCODER_COUNT);		//NB: this overrides the 3mxel default
    mLock = false;

    delete config;
    std::cout << "Motor " << mMotorId << " initialising completed." << std::endl;
}
