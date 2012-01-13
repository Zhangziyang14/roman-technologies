/*
 * HeadMotorHander.cpp
 *
 *  Created on: 2012-01-13
 *      Author: wilson
 */

#include <head/HeadMotorHandler.h>

/**
 * Publishes head's current position
 */
void HeadMotorHandler::publishHeadPosition()
{
	mCurrentPose.orientation.x = mDynamixel.getPosition();
	mPositionPub.publish(mCurrentPose);
}

/**
 * Controls the motors based on the received position.
 */
void HeadMotorHandler::positionCB(const geometry_msgs::Pose& msg)
{
	mDynamixel.setPosition(msg.orientation.x);
}

/**
 * Initialise MotorHandler and its attributes.
 */
void HeadMotorHandler::init(char *path)
{
	//Initialise publishers
	mPositionPub = mNodeHandle.advertise<geometry_msgs::Pose>("/headPositionFeedbackTopic", 1);

	//Initialise subscribers
	mPositionSub 	= mNodeHandle.subscribe("/headPositionTopic", 10, &HeadMotorHandler::positionCB, this);

	mDynamixel.init(path);

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
