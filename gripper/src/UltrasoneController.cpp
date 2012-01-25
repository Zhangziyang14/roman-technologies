/*
 * UltrasoneController.cpp
 *
 *  Created on: Oct 11, 2011
 *      Author: hans
 */

#include "gripper/UltrasoneController.h"

void UltrasoneController::init()
{
	Controller::init();

	mPingCommand_pub 	= mNodeHandle.advertise<std_msgs::Bool>("pingActivateTopic", 1);

	// initialize subscribers
	mSensor_sub 	= mNodeHandle.subscribe("pingFeedbackTopic", 10, &UltrasoneController::readSensorDataCB, this);
	mCommand_sub 	= mNodeHandle.subscribe("/cmd_gripper", 1, &UltrasoneController::commandCB, this);
}

/**
 * Opens or closes the gripper
 */
void UltrasoneController::commandCB(const std_msgs::Bool& msg)
{
	std_msgs::Bool bool_msg;

	bool_msg.data = !msg.data;

	mPingCommand_pub.publish(bool_msg);

	if (msg.data)
		setGripper(msg.data);
}

/**
 * Called when new sensor data is made available.
 */
void UltrasoneController::readSensorDataCB(const std_msgs::UInt16& msg)
{
	ROS_INFO("I heard: [%d]", msg.data);

	// default message, open the gripper
	gripper::MotorControl mc;
	mc.modeStr = "torque";
	mc.value = AUTOMATIC_GRIPPER_TORQUE;
	mc.waitTime = GRIPPER_WAIT_TIME;

	// Is the distance smaller than 10cm and are we not grabbing the object yet? Then close the gripper
	if (msg.data < CLOSE_GRIPPER_DISTANCE && mGripperState != GS_CLOSED)
	{
		setGripper(false);
	}
}

int main(int argc, char **argv)
{
	// init ros and controller
	ros::init(argc, argv, "UltrasoneController");
	UltrasoneController controller;
	controller.init();
	ros::spin();
	return 0;
}
