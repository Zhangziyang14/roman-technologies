#include <Motor.h>

using namespace std;

/**
  * Sets the mode of the motor
 */
void Motor::setMode(int mode){
    switch(mode){
    case SPEED_MODE:
        cmode = CM_SPEED_MODE;
        motor_->set3MxlMode(SPEED_MODE);
        cout << "Changed to speed mode" << endl;
        break;

    case TORQUE_MODE:
        cmode = CM_TORQUE_MODE;
        motor_->set3MxlMode(TORQUE_MODE);
        cout << "Changed to torque mode" << endl;
        break;

    case CURRENT_MODE:
        cmode = CM_CURRENT_MODE;
        motor_->set3MxlMode(CURRENT_MODE);
        cout << "Changed to current mode" << endl;
        break;

    case STOP_MODE:
        cmode = CM_STOP_MODE;
        cout << "Changed to stop mode" << endl;
        motor_->set3MxlMode(STOP_MODE);
        break;

    case POSITION_MODE:
        cmode = CM_POSITION_MODE;
        cout << "Changed to position mode" << endl;
        motor_->set3MxlMode(POSITION_MODE);
        break;

    case SEA_MODE:
        cmode = CM_SEA_MODE;
        cout << "Changed to sea mode" << endl;
        motor_->set3MxlMode(SEA_MODE);
        break;

    case PWM_MODE:
        cmode = CM_PWM_MODE;
        cout << "Changed to pwm mode" << endl;
        motor_->set3MxlMode(PWM_MODE);
        break;

    case TEST_MODE:
        cmode = CM_TEST_MODE;
        cout << "Changed to test mode" << endl;
        motor_->set3MxlMode(TEST_MODE);
        break;
    }
}


/**
  * Sets acceleration of the motor when in SPEED_MODE
 */
void Motor::setAcceleration(double acceleration){
    if(cmode != SPEED_MODE){
        cout << "Motor not in speed mode" << endl;
        setMode(SPEED_MODE);
    }

    motor_->setAcceleration(acceleration);
}

/**
  * Sets speed of the motor when in SPEED_MODE
 */
void Motor::setSpeed(double speed){

    if(speed == 0){
        setMode(STOP_MODE);
    }
    else{
        if(cmode != SPEED_MODE){
            cout << "Motor not in speed mode" << endl;
            setMode(SPEED_MODE);
        }

        setAcceleration(10);
        motor_->setSpeed(speed);
    }
}

int Motor::getMode(){
    switch(cmode){
    case CM_NONE:
        return -1;

    case CM_SPEED_MODE:
        return SPEED_MODE;

    case CM_TORQUE_MODE:
        return TORQUE_MODE;

    case CM_CURRENT_MODE:
        return CURRENT_MODE;

    case CM_STOP_MODE:
        return STOP_MODE;

    case CM_POSITION_MODE:
        return POSITION_MODE;

    case CM_SEA_MODE:
        return SEA_MODE;

    case CM_PWM_MODE:
        return PWM_MODE;

    case CM_TEST_MODE:
        return TEST_MODE;
    }

    return -2;
}

int Motor::getID(){
    return engine_number;
}

/**
 * Initalize Motor and its attributes.
*/
void Motor::init(char *path){
    CDxlConfig *config = new CDxlConfig();

    // find the connection to the motor on the 3mXl board
    if (path){
        cout << "Using shared_serial"<< endl;
        motor_ = new C3mxlROS(path);
    }
    else{
        cout << "Using direct connection" << endl;
        motor_ = new C3mxl();

        serial_port_.port_open("/dev/mobile_manipulator", LxSerial::RS485_FTDI);
        serial_port_.set_speed(LxSerial::S921600);
        motor_->setSerialPort(&serial_port_);
    }

    // initialize the motor
    motor_->setConfig(config->setID(engine_number));
    motor_->init(false);
    setMode(STOP_MODE);

    delete config;
    cout << "Motor " << engine_number << " initializing completed." << endl;
}
