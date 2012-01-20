#include "std_msgs/String.h"
#include <ros/ros.h>
#include <threemxl/example.h>
#include <threemxl/C3mxlROS.h>
#include <gripper/MotorHandler.h>
#include <iostream>

/**
 * Called when a motor command is received over the motor topic.
 */
void MotorHandler::chatterCallback(const gripper::MotorControlPtr& mc)
{
    // print out message for debugging
    std::stringstream ss("");
    ss << mc->modeStr << " " << mc->value << " " << mc->waitTime;
    ROS_INFO("Received message: %s", ss.str().c_str());

    // parse the control mode
    ControlMode mode = CM_NONE;
    if (strcmp(mc->modeStr.c_str(), "torque") == 0)
        mode = CM_TORQUE_MODE;
    else if (strcmp(mc->modeStr.c_str(), "current") == 0)
        mode = CM_CURRENT_MODE;
    else if (strcmp(mc->modeStr.c_str(), "stop") == 0)
        mode = CM_STOP_MODE;

    // some wrong message?
    if (mode == CM_NONE)
        return;

    // Are we changing modes? Then let the motor know.
    if (mode != cmode)
    {
        motor_->set3MxlMode(mode);
        cmode = mode;
    }
    // pass the value to the motor
    switch (mode)
    {
    case CM_TORQUE_MODE:
        motor_->setTorque(mc->value);
        break;
    case CM_CURRENT_MODE:
        motor_->setCurrent(mc->value);
        break;
    default:
        break;
    }

    // do we need to sleep for a bit and then disable the motor ? (Use threads?)
    if (mc->waitTime)
    {
        usleep(mc->waitTime * 1000);
        motor_->set3MxlMode(CM_STOP_MODE);
        motor_->set3MxlMode(cmode);
    }
}

/**
 * Initalize MotorHandler and its attributes.
*/
void MotorHandler::init(char *path)
{
    CDxlConfig *config = new CDxlConfig();

    // find the connection to the motor on the 3mXl board
    if (path)
    {
        ROS_INFO("Using shared_serial");
        motor_ = new C3mxlROS(path);
    }
    else
    {
        ROS_INFO("Using direct connection");
        motor_ = new C3mxl();
    
        serial_port_.port_open("/dev/roman/threemxl", LxSerial::RS485_FTDI);
        serial_port_.set_speed(LxSerial::S921600);
        motor_->setSerialPort(&serial_port_);
    }

    // initialize the motor
    motor_->setConfig(config->setID(109));
    motor_->init(false);
    motor_->set3MxlMode(TORQUE_MODE);

    delete config;
    ROS_INFO("Initializing completed."); 
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "listener");

    ros::NodeHandle n;

    char *path="serial";
    if (argc == 2)
        path = argv[1];
 
    MotorHandler motorHandler;
    motorHandler.init(path);

    ros::Subscriber sub = n.subscribe("motorTopic", 1, &MotorHandler::chatterCallback, &motorHandler);

    ros::spin();


    return 0;
}

