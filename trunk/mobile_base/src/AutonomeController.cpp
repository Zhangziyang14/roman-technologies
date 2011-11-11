/*
 * AutonomeController.cpp
 *
 *  Created on: Oct 20, 2011
 *      Author: hans
 */

#include "AutonomeController.h"

std_msgs::Float64 position_msg;
mobile_base::DisableMotor disable_msg;

/**
 * Receives data from ultrasone sensors and determines if forward or backward motion should be disabled.
 */
void AutonomeController::ultrasoneFeedbackCB(const mobile_base::sensorFeedback &msg)
{

	// did the sensors on the front detect anything dangerously close?
	disable_msg.disableForward = (msg.frontCenter > 0 && msg.frontCenter < EMERGENCY_STOP_DISTANCE) ||
			(msg.frontLeft > 0 && msg.frontLeft < EMERGENCY_STOP_DISTANCE) ||
			(msg.frontRight> 0 && msg.frontRight < EMERGENCY_STOP_DISTANCE);

	// did the sensors on the back detect anything dangerously close?
	disable_msg.disableBackward = (msg.rearCenter > 0 && msg.rearCenter < EMERGENCY_STOP_DISTANCE) ||
			(msg.rearLeft > 0 && msg.rearLeft < EMERGENCY_STOP_DISTANCE) ||
			(msg.rearRight> 0 && msg.rearRight < EMERGENCY_STOP_DISTANCE);

	//ROS_INFO("[AutonomeController] %s Forward.", disable_msg.disableForward ? "Disable" : "Enable");
	//ROS_INFO("[AutonomeController] %s Backward.", disable_msg.disableBackward ? "Disable" : "Enable");

	mDisableMotor_pub.publish(disable_msg);
}

void AutonomeController::bumperFeedbackCB(const std_msgs::UInt8 &msg)
{

	// did the robot collide with something in the front?
	disable_msg.disableForward = msg.data == BUMPER_FRONT;

	// did the robot collide with something in the back?
	disable_msg.disableBackward = (msg.data == BUMPER_REAR) ||  (msg.data == BUMPER_REAR_LEFT)
											|| (msg.data == BUMPER_REAR_RIGHT);

	mDisableMotor_pub.publish(disable_msg);

	if(disable_msg.disableForward)
		position_msg.data = -0.5;
	else
		position_msg.data = 0.5;

	mMovement_pub.publish(position_msg);
}

/**
 * Initialises this controller.
 */
void AutonomeController::init()
{
	// initialise subscribers
	mSensorFeedback_sub = mNodeHandle.subscribe("/sensorFeedbackTopic", 10, &AutonomeController::ultrasoneFeedbackCB, this);
	mBumperFeedback_sub = mNodeHandle.subscribe("/bumperFeedbackTopic", 10, &AutonomeController::bumperFeedbackCB, this);

	// initialise publishers
	mDisableMotor_pub = mNodeHandle.advertise<mobile_base::DisableMotor>("/disableMotorTopic", 10);
	mMovement_pub = mNodeHandle.advertise<std_msgs::Float64>("/positionTopic", 1);

	ROS_INFO("AutonomeController initialised");
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "AutonomeController");

	char *path=NULL;
	if (argc == 2)
		path = argv[1];

	AutonomeController controller;
	controller.init();

	ros::spin();
	return 0;
}
