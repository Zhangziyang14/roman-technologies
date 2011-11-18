/*
 * SafeKeeper.cpp
 *
 *  Created on: Oct 20, 2011
 *      Author: hans
 */

#include "SafeKeeper.h"

std_msgs::Float64 position_msg;
mobile_base::BumperDisableMotor disableBumper_msg;

/**
 * Checks whether motors were already disabled by ultrasone sensors
 */
void SafeKeeper::DisabledByUltrasoneCB(const mobile_base::DisableMotor &msg)
{
	mFrontDisabledByUltrasone = msg.disableBackward;
	mRearDisabledByUltrasone = msg.disableForward;
}

/**
 * Checks whether robot collided with something and determines if forward or backward motion should be disabled
 * Also moves the robot 0.5 [m] (if possible) from the object.
 */
void SafeKeeper::bumperFeedbackCB(const std_msgs::UInt8 &msg)
{

	// did the robot collide with something in the front?
	disableBumper_msg.disableForward = msg.data == BUMPER_FRONT;

	// did the robot collide with something in the back?
	disableBumper_msg.disableBackward = (msg.data == BUMPER_REAR);

	mBumperDisableMotor_pub.publish(disableBumper_msg);

	if(mBumperState != BUMPER_FRONT && disableBumper_msg.disableForward)
	{
		mBumperState = BUMPER_FRONT;
		position_msg.data = -SAFE_DISTANCE;
	}

	if(mBumperState != BUMPER_REAR && disableBumper_msg.disableBackward)
	{
		mBumperState = BUMPER_REAR;
		position_msg.data = SAFE_DISTANCE;
	}

	if(mBumperState == BUMPER_REAR)
		mMovement_pub.publish(position_msg);

	else if(mBumperState == BUMPER_FRONT)
		mMovement_pub.publish(position_msg);

	disableBumper_msg.disableForward = false;
	disableBumper_msg.disableBackward = false;

	mBumperState = BUMPER_NONE;

	mBumperDisableMotor_pub.publish(disableBumper_msg);
}

/**
 * Initialises this controller.
 */
void SafeKeeper::init()
{
	// initialise subscribers
	mBumperFeedback_sub = mNodeHandle.subscribe("/bumperFeedbackTopic", 10, &SafeKeeper::bumperFeedbackCB, this);
	mUltrasoneDisableMotor_sub = mNodeHandle.subscribe("disableMotorTopic", 10, &SafeKeeper::DisabledByUltrasoneCB, this);	

	// initialise publishers
	mBumperDisableMotor_pub = mNodeHandle.advertise<mobile_base::BumperDisableMotor>("/bumperDisableMotorTopic", 10);
	mMovement_pub = mNodeHandle.advertise<std_msgs::Float64>("/positionTopic", 1);

	mFrontDisabledByUltrasone = false;
	mRearDisabledByUltrasone = false;

	ROS_INFO("SafeKeeper initialised");
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "SafeKeeper");

	char *path=NULL;
	if (argc == 2)
		path = argv[1];

	SafeKeeper safeKeeper;
	safeKeeper.init();

	ros::spin();
	return 0;
}

