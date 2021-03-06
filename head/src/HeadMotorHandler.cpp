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
	mCurrentPose.pitch = mPitch.getPosition() - PITCH_OFFSET;
	mCurrentPose.yaw = mYaw.getPosition() - YAW_OFFSET;

	tf::Quaternion orientation;
	orientation.setRPY(-mCurrentPose.pitch, 0.0, mCurrentPose.yaw);
	mKinectTF.setRotation(orientation);
	mKinectTFBroadcaster.sendTransform(tf::StampedTransform(mKinectTF, ros::Time::now(), "head_frame", "kinect_normal_axis_frame"));

	if (mPositionPub.getNumSubscribers() == 0)
		return;

	mPositionPub.publish(mCurrentPose);
/*
	if((mCurrentSpeed.pitch < 0 && mCurrentPose.pitch  > PITCH_LOWER_LIMIT) ||
			(mCurrentSpeed.pitch > 0 && mCurrentPose.pitch < PITCH_UPPER_LIMIT))
	{
		mPitch.setSpeed(0.0);
	}

	if((mCurrentSpeed.yaw < 0 && mCurrentPose.yaw  > YAW_LOWER_LIMIT) ||
			(mCurrentSpeed.yaw > 0 && mCurrentPose.yaw < YAW_UPPER_LIMIT))
	{
		mYaw.setSpeed(0.0);
	}
*/
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
void HeadMotorHandler::speedCB(const nero_msgs::PitchYaw &msg)
{
	double avgPitch = (PITCH_LOWER_LIMIT + PITCH_UPPER_LIMIT) / 2;
	double avgYaw = (YAW_LOWER_LIMIT + YAW_UPPER_LIMIT) / 2;
	double pitchScale = 1.0 - (fabs(avgPitch - std::min(PITCH_UPPER_LIMIT, std::max(PITCH_LOWER_LIMIT, (double)mCurrentPose.pitch))) / (PITCH_UPPER_LIMIT - avgPitch));
	double yawScale = 1.0 - (fabs(avgYaw - std::min(YAW_UPPER_LIMIT, std::max(YAW_LOWER_LIMIT, (double)mCurrentPose.yaw))) / (YAW_UPPER_LIMIT - avgYaw));

	if ((msg.pitch > 0.0 && mCurrentPose.pitch < avgPitch) || (msg.pitch < 0.0 && mCurrentPose.pitch > avgPitch))
		pitchScale = 1.0;
	if ((msg.yaw > 0.0 && mCurrentPose.yaw < avgYaw) || (msg.yaw < 0.0 && mCurrentPose.yaw > avgYaw))
		yawScale = 1.0;

	mPitch.setSpeed(msg.pitch * pitchScale);
	mYaw.setSpeed(msg.yaw * yawScale);
}

/**
 * Controls the motors based on the received position.
 */
void HeadMotorHandler::positionCB(const nero_msgs::PitchYaw &msg)
{
	double pitch	= msg.pitch;
	double yaw 		= msg.yaw;

	if (isnan(pitch) || isnan(yaw))
		return;

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
	mPositionPub	= mNodeHandle.advertise<nero_msgs::PitchYaw>("/headPositionFeedbackTopic", 1);
	mSpeedPub		= mNodeHandle.advertise<nero_msgs::PitchYaw>("/headSpeedFeedbackTopic", 1);

	//Initialise subscribers
	mPositionSub	= mNodeHandle.subscribe("/headPositionTopic", 10, &HeadMotorHandler::positionCB, this);
	mSpeedSub		= mNodeHandle.subscribe("/head/cmd_vel", 10, &HeadMotorHandler::speedCB, this);

	mPitch.init(path);
	mYaw.init(path);

	mKinectTF.setOrigin(tf::Vector3(-0.03, 0.055, 0.12));

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
