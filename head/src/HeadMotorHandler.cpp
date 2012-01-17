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
	mCurrentPose.orientation.x = mTilt.getPosition();
	mCurrentPose.orientation.z = mPan.getPosition();
	mPositionPub.publish(mCurrentPose);
}

/**
 * Controls the motors based on the received position.
 */
void HeadMotorHandler::positionCB(const geometry_msgs::Pose& msg)
{

	double tilt = msg.orientation.x;
	double pan 	= msg.orientation.z;
	//servo.setPosition(msg.orientation.z);
	if(msg.orientation.x > TILT_UPPER_LIMIT)
		tilt = TILT_UPPER_LIMIT;

	if(msg.orientation.x < TILT_LOWER_LIMIT)
		tilt = TILT_LOWER_LIMIT;

	if(msg.orientation.z > PAN_UPPER_LIMIT)
		pan = PAN_UPPER_LIMIT;

	if(msg.orientation.z < PAN_LOWER_LIMIT)
		pan = PAN_LOWER_LIMIT;

	mTilt.setPosition(tilt);
	mPan.setPosition(pan);
}

/**
 * Initialise MotorHandler and its attributes.
 */
void HeadMotorHandler::init(char *path)
{
	//Initialise publishers
	mPositionPub	= mNodeHandle.advertise<geometry_msgs::Pose>("/headPositionFeedbackTopic", 1);

	//Initialise subscribers
	mPositionSub	= mNodeHandle.subscribe("/headPositionTopic", 10, &HeadMotorHandler::positionCB, this);

	mTilt.init(path);
	mPan.init(path);

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
