/*
 * HeadMotorHander.cpp
 *
 *  Created on: 2012-01-13
 *      Author: wilson
 */

#include <head/HeadMotorHandler.h>

//TODO: INITIALISE AND STOPHEAD SAFELY
//TODO: USE QUATERNION PROPERLY

/**
 * Publishes head's current position
 */
void HeadMotorHandler::publishHeadPosition()
{
	mCurrentPose.pitch = mPitch.getPosition() - PITCH_OFFSET;
	mCurrentPose.yaw = mYaw.getPosition() - YAW_OFFSET;
	mPositionPub.publish(mCurrentPose);
}

/**
 * Controls the motors based on the received position.
 */
void HeadMotorHandler::positionCB(const head::PitchYaw &msg)
{

	double pitch	= msg.pitch;
	double yaw 		= msg.yaw;

	if(msg.pitch > PITCH_UPPER_LIMIT)
		pitch = PITCH_UPPER_LIMIT;

	if(msg.pitch < PITCH_LOWER_LIMIT)
		pitch = PITCH_LOWER_LIMIT;

	if(msg.yaw > YAW_UPPER_LIMIT)
		yaw = YAW_UPPER_LIMIT;

	if(msg.yaw < YAW_LOWER_LIMIT)
		yaw = YAW_LOWER_LIMIT;

	mPitch.setPosition(pitch + PITCH_OFFSET);
	mYaw.setPosition(yaw + YAW_OFFSET);
}

/**
 * Initialise MotorHandler and its attributes.
 */
void HeadMotorHandler::init(char *path)
{
	//Initialise publishers
	mPositionPub	= mNodeHandle.advertise<head::PitchYaw>("/headPositionFeedbackTopic", 1);

	//Initialise subscribers
	mPositionSub	= mNodeHandle.subscribe("/headPositionTopic", 10, &HeadMotorHandler::positionCB, this);

	mPitch.init(path);
	mYaw.init(path);

	ROS_INFO("Initialising completed.");
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "MotorHandler");

	char *path=NULL;
	if (argc == 2)
		path = argv[1];

	HeadMotorHandler motorHandler;
	motorHandler.init(path);

	while(ros::ok())
	{
		motorHandler.publishHeadPosition();
		ros::spinOnce();
	}

	return 0;
}
