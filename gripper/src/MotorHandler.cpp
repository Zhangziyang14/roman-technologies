#include "std_msgs/String.h"
#include <ros/ros.h>
#include <threemxl/example.h>
#include <threemxl/C3mxlROS.h>
#include <gripper/MotorHandler.h>
#include <iostream>

/**
 * Called when a motor command is received over the motor topic.
 */
void MotorHandler::gripperCB(const gripper::MotorControl& mc)
{
    // print out message for debugging
    std::stringstream ss("");
    ss << mc.modeStr << " " << mc.value << " " << mc.waitTime;
    ROS_INFO("Received message: %s", ss.str().c_str());

    // parse the control mode
    ControlMode mode = CM_NONE;
    if (strcmp(mc.modeStr.c_str(), "torque") == 0)
        mode = CM_TORQUE_MODE;
    else if (strcmp(mc.modeStr.c_str(), "current") == 0)
        mode = CM_CURRENT_MODE;
    else if (strcmp(mc.modeStr.c_str(), "stop") == 0)
        mode = CM_STOP_MODE;

    // some wrong message?
    if (mode == CM_NONE)
        return;

    // Are we changing modes? Then let the motor know.
    if (mode != cmode)
    {
        mGripperMotor.setMode(cmode);
        cmode = mode;
    }
    // pass the value to the motor
    switch (mode)
    {
    case CM_TORQUE_MODE:
    		mGripperMotor.setTorque(mc.value);
        break;

    case CM_CURRENT_MODE:
        mGripperMotor.setCurrent(mc.value);
        break;

    default:
        break;
    }

    // Do we need to sleep for a bit and then disable the motor ? (Use threads?)
    if (mc.waitTime)
    {
        usleep(mc.waitTime * 1000);
        mGripperMotor.setMode(CM_STOP_MODE);
        mGripperMotor.setMode(cmode);
    }
}

/**
 * Initalize MotorHandler and its attributes.
*/
void MotorHandler::init(char *path)
{
	//Initialise subscribers
	mMotorSub = mNodeHandle.subscribe("/gripperMotorTopic", 1, &MotorHandler::gripperCB, this);

	mGripperMotor.init(path);
	ROS_INFO("Initialising completed.");
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "gripperMotorHandler");

    char *path=NULL;
    if (argc == 2)
        path = argv[1];
 
    MotorHandler motorHandler;
    motorHandler.init(path);

    ros::spin();
    return 0;
}

