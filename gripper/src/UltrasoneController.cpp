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

	mEmotion_pub = mNodeHandle.advertise<std_msgs::ColorRGBA>("rgbTopic", 1);

	// initialize subscribers
	mSensor_sub = mNodeHandle.subscribe("pingFeedbackTopic", 10, &UltrasoneController::readSensorDataCB, this);
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
		mc.value = -mc.value;
		mMotor_pub.publish(mc);
		mGripperState = GS_CLOSED;

		std_msgs::ColorRGBA color;
		color.r = 0;
		color.g = 255;
		color.b = 0;
		color.a = 0;

		mEmotion_pub.publish(color);

	}
	// Is the distance greater than 15cm and are we grabbing the object, then open the gripper.
	else if (msg.data > OPEN_GRIPPER_DISTANCE && mGripperState != GS_OPEN)
	{
		mMotor_pub.publish(mc);
		mGripperState = GS_OPEN;

		std_msgs::ColorRGBA color;
		color.r = 0;
		color.g = 0;
		color.b = 0;
		color.a = 0;

		mEmotion_pub.publish(color);
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
