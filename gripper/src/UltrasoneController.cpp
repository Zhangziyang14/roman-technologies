/*
 * UltrasoneController.cpp
 *
 *  Created on: Oct 11, 2011
 *      Author: hans
 */

#include "UltrasoneController.h"

void UltrasoneController::init()
{
	Controller::init();

    // initialize subscribers
    mSensor_sub = mNodeHandle.subscribe("sensorFeedbackTopic", 10, &UltrasoneController::readSensorDataCB, this);
}

/**
 * Called when new sensor data is made available.
*/
void UltrasoneController::readSensorDataCB(const gripper::DistancePtr& msg)
{
    ROS_INFO("I heard: [%d]", msg->distance);

    // default message, open the gripper
    gripper::MotorControl mc;
    mc.modeStr = "torque";
    mc.value = AUTOMATIC_GRIPPER_TORQUE;
    mc.waitTime = GRIPPER_WAIT_TIME;

    // Is the distance smaller than 10cm and are we not grabbing the object yet? Then close the gripper
    if (msg->distance < CLOSE_GRIPPER_DISTANCE && mGripperState != GS_CLOSED)
    {
        mc.value = -mc.value;
        mMotor_pub.publish(mc);
        mGripperState = GS_CLOSED;
    }
    // Is the distance greater than 15cm and are we grabbing the object, then open the gripper.
    else if (msg->distance > OPEN_GRIPPER_DISTANCE && mGripperState != GS_OPEN)
    {
        mMotor_pub.publish(mc);
        mGripperState = GS_OPEN;
    }
}

int main(int argc, char **argv)
{
    // init ros and controller
    ros::init(argc, argv, "controller");
    UltrasoneController controller;
    controller.init();
    ros::spin();
    return 0;
}
