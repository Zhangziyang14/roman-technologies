/*
 * HeadMotorHander.cpp
 *
 *  Created on: 2012-01-13
 *      Author: wilson
 */

#include <head/HeadMotorHandler.h>

//TODO: INITIALISE AND STOP HEAD SAFELY

/**
 * Publishes head's current position
 */
void HeadMotorHandler::publishHeadPosition()
{
	mCurrentPose.pitch = mPitch.getPosition() - PITCH_OFFSET;
	mCurrentPose.yaw = mYaw.getPosition() - YAW_OFFSET;
	mPositionPub.publish(mCurrentPose);

	btQuaternion orientation;
	orientation.setRPY(mCurrentPose.pitch, 0.0, -mCurrentPose.yaw);
	mKinectTF.setRotation(orientation);
	mKinectTFBroadcaster.sendTransform(tf::StampedTransform(mKinectTF, ros::Time::now(), "kinect_normal_axis_frame", "head_frame"));
}

/**
 * Publishes head's current speed
 */
void HeadMotorHandler::publishHeadSpeed()
{
	mCurrentSpeed.pitch = mPitch.getRotationSpeed();
	mCurrentSpeed.yaw = mYaw.getRotationSpeed();
	mSpeedPub.publish(mCurrentSpeed);
}

/**
 * Controls the motors based on the received position.
 */
void HeadMotorHandler::positionCB(const head::PitchYaw &msg)
{
	double pitch	= msg.pitch;
	double yaw 		= msg.yaw;

	if (isnan(pitch) || isnan(yaw))
	{
		ROS_WARN("Received nan in positionCb.");
		return;
	}

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
	mSpeedPub		= mNodeHandle.advertise<head::PitchYaw>("/headSpeedFeedbackTopic", 1);

	//Initialise subscribers
	mPositionSub	= mNodeHandle.subscribe("/headPositionTopic", 10, &HeadMotorHandler::positionCB, this);

	mPitch.init(path);
	mYaw.init(path);

	mKinectTF.setOrigin(tf::Vector3(0.03, -0.055, -0.12));

	ROS_INFO("Initialising completed.");
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "HeadMotorHandler");

	char *path=NULL;
	if (argc == 2)
		path = argv[1];

	HeadMotorHandler motorHandler;
	motorHandler.init(path);

	int sleep_rate;
	motorHandler.getNodeHandle()->param<int>("node_sleep_rate", sleep_rate, 50);
	ros::Rate sleep(sleep_rate);

	while(ros::ok())
	{
		motorHandler.publishHeadSpeed();
		motorHandler.publishHeadPosition();
		sleep.sleep();
		ros::spinOnce();
	}

	return 0;
}
