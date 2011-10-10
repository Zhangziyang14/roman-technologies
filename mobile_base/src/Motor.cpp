#include <Motor.h>

using namespace std;

double &PID::operator[](unsigned int i)
{
	switch (i)
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
	cout << "Changed to ";
    switch (mode)
    {
    case SPEED_MODE: cout << "speed"; break;
    case TORQUE_MODE: cout << "torque"; break;
    case CURRENT_MODE: cout << "current"; break;
    case STOP_MODE: cout << "stop"; break;
    case POSITION_MODE: cout << "position"; break;
    case SEA_MODE: cout << "sea"; break;
    case PWM_MODE: cout << "pwm"; break;
    case TEST_MODE: cout << "test"; break;
    default : break;
    }
    cout << " mode." << endl;
}


/**
  * Sets acceleration of the motor when in SPEED_MODE
 */
void Motor::setAcceleration(double acceleration)
{
    if (cmode != CM_SPEED_MODE)
    {
        cout << "Motor not in speed mode" << endl;
        setMode(CM_SPEED_MODE);
    }

    motor_->setAcceleration(acceleration);
}

/**
  * Sets speed of the motor when in SPEED_MODE
 */
void Motor::setSpeed(double speed)
{
	setAcceleration(5);
    motor_->setSpeed(speed);
}

/**
  * Gets speed of the motor when in SPEED_MODE
 */
double Motor::getRotationSpeed()
{
	motor_->getState();
	motor_->getStatus();

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

/**
 * Initalize Motor and its attributes.
*/
void Motor::init(char *path)
{
    CDxlConfig *config = new CDxlConfig();

    // find the connection to the motor on the 3mXl board
    if (path)
    {
        cout << "Using shared_serial"<< endl;
        motor_ = new C3mxlROS(path);
    }
    else
    {
        cout << "Using direct connection" << endl;
        motor_ = new C3mxl();

        serial_port_.port_open("/dev/motorusb", LxSerial::RS485_FTDI);
        serial_port_.set_speed(LxSerial::S921600);
        motor_->setSerialPort(&serial_port_);    
    }

    // initialize the motor
    motor_->setConfig(config->setID(mMotorId));
    motor_->init(false);
    setMode(CM_STOP_MODE);

    delete config;
    cout << "Motor " << mMotorId << " initializing completed." << endl;
}
