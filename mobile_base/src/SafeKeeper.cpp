/*
 * SafeKeeper.cpp
 *
 *  Created on: Oct 20, 2011
 *      Author: hans
 */

#include "mobile_base/SafeKeeper.h"

/**
 * Checks whether robot collided with something and determines if forward or backward motion should be disabled
 * Also moves the robot 0.5 [m] (if possible) from the object.
 */
void SafeKeeper::bumperFeedbackCB(const std_msgs::UInt8 &msg)
{
	mobile_base::position position_msg;
	// did the robot collide with something in the front?
	mDisableForward = msg.data == BUMPER_FRONT;

	// did the robot collide with something in the back?
	mDisableBackward = msg.data == BUMPER_REAR;

	if(mBumperState != BUMPER_FRONT && mDisableForward)
	{
		mBumperState = BUMPER_FRONT;
		position_msg.left   = -SAFE_DISTANCE;
		position_msg.right  = -SAFE_DISTANCE;
	}

	if(mBumperState != BUMPER_REAR && mDisableBackward)
	{
		mBumperState = BUMPER_REAR;
		position_msg.left   = SAFE_DISTANCE;
		position_msg.right  = SAFE_DISTANCE;
	}

	mMovement_pub.publish(position_msg);

	mDisableForward	 = false;
	mDisableBackward = false;

	mBumperState = BUMPER_NONE;
}

/**
 * Listens to ultrasone sensor data and stops the base when something suddenly gets in the way of the base.
 * Also moves the base a bit back from where it came.
 */
void SafeKeeper::ultrasoneFeedbackCB(const mobile_base::sensorFeedback& msg)
{
	if(msg.frontLeftCenter - mFrontLeftCenter < 0 && msg.frontRightCenter - mFrontRightCenter < 0)
	{
		mobile_base::position position_msg;
		position_msg.left   = -SAFE_DISTANCE;
		position_msg.right  = -SAFE_DISTANCE;
		mMovement_pub.publish(position_msg);
	}
	else if(msg.rearLeft - mRearLeft < 0 && msg.rearRight - mRearRight < 0)
	{
		mobile_base::position position_msg;
		position_msg.left   = SAFE_DISTANCE;
		position_msg.right  = SAFE_DISTANCE;
		mMovement_pub.publish(position_msg);
	}

	mFrontLeftCenter 	= msg.frontLeftCenter;
	mFrontRightCenter	= msg.frontRightCenter;

	mRearLeft			= msg.rearLeft;
	mRearRight			= msg.rearRight;
}

/**
 * Initialises this controller.
 */
void SafeKeeper::init()
{
	// initialise subscribers
	mBumperFeedback_sub = mNodeHandle.subscribe("/bumperFeedbackTopic", 10, &SafeKeeper::bumperFeedbackCB, this);
	mUltrasone_sub		= mNodeHandle.subscribe("/sensorFeedbackTopic", 10, &SafeKeeper::ultrasoneFeedbackCB, this);

	// initialise publishers
	mMovement_pub 		= mNodeHandle.advertise<mobile_base::position>("/positionTopic", 1);

	mDisableForward	 	= false;
	mDisableBackward 	= false;

	mFrontLeftCenter 	= 0;
	mFrontRightCenter 	= 0;

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
