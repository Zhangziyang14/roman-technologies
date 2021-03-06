/*
 * SafeKeeper.cpp
 *
 *  Created on: Oct 20, 2011
 *      Author: hans
 */

#include "mobile_base/SafeKeeper.h"

using namespace nero_msgs;

/**
 * Checks whether robot collided with something and determines if forward or backward motion should be disabled
 * Also moves the robot 0.5 [m] (if possible) from the object.
 */
void SafeKeeper::bumperFeedbackCB(const std_msgs::UInt8 &msg)
{
	nero_msgs::MotorPosition position_msg;
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

//	mMovement_pub.publish(position_msg);

	mDisableForward	 = false;
	mDisableBackward = false;

	mBumperState = BUMPER_NONE;

	if (position_msg.left && position_msg.right)
	{
		if (mBumperStartTime == 0.0)
			mBumperStartTime = ros::Time::now().toSec();

		if (ros::Time::now().toSec() - mBumperStartTime > ERROR_STATE_BUMPER_TIME)
		{
			mBumperStartTime = ros::Time::now().toSec();
			mBumperCount = 0;
		}

		mBumperCount++;
		if (mBumperCount >= ERROR_STATE_BUMPER_COUNT)
		{
			std_msgs::UInt8 msg;
			msg.data = nero_msgs::Emotion::ERROR;
			mEmotion_pub.publish(msg);
			mBumperCount = 0;
			mBumperStartTime = ros::Time::now().toSec();
		}
	}
}

/**
 * Listens to ultrasone sensor data and stops the base when something suddenly gets in the way of the base.
 * Also moves the base a bit back from where it came.
 */
void SafeKeeper::ultrasoneFeedbackCB(const nero_msgs::SensorFeedback& msg)
{
	if(msg.data[SensorFeedback::SENSOR_FRONT_CENTER_LEFT] - mSensorData[SensorFeedback::SENSOR_FRONT_CENTER_LEFT] < 0 &&
			msg.data[SensorFeedback::SENSOR_FRONT_CENTER_RIGHT] - mSensorData[SensorFeedback::SENSOR_FRONT_CENTER_RIGHT] < 0 && mCurrentSpeed.linear.x > 0)
	{
		nero_msgs::MotorPosition position_msg;
		position_msg.left   = 0;
		position_msg.right  = 0;
//		mMovement_pub.publish(position_msg);
	}
	else if((msg.data[SensorFeedback::SENSOR_REAR_LEFT] - mSensorData[SensorFeedback::SENSOR_REAR_LEFT] < 0 ||
			msg.data[SensorFeedback::SENSOR_REAR_RIGHT] - mSensorData[SensorFeedback::SENSOR_REAR_RIGHT] < 0) && mCurrentSpeed.linear.x < 0)
	{
		nero_msgs::MotorPosition position_msg;
		position_msg.left   = 0;
		position_msg.right  = 0;
//		mMovement_pub.publish(position_msg);
	}

	mSensorData = msg.data;
}

/**
 * Keeps track of current speed of mobile base
 */
void SafeKeeper::speedFeedbackCB(const geometry_msgs::Twist &msg)
{
	mCurrentSpeed = msg;
}

/**
 * Initialises this controller.
 */
void SafeKeeper::init()
{
	// initialise subscribers
	mBumperFeedback_sub = mNodeHandle.subscribe("/bumperFeedbackTopic", 10, &SafeKeeper::bumperFeedbackCB, this);
	mUltrasone_sub		= mNodeHandle.subscribe("/sensorFeedbackTopic", 10, &SafeKeeper::ultrasoneFeedbackCB, this);
//	mSpeed_sub			= mNodeHandle.subscribe("/speedFeedbackTopic", 10, &SafeKeeper::speedFeedbackCB, this);

	// initialise publishers
	mMovement_pub 		= mNodeHandle.advertise<nero_msgs::MotorPosition>("/positionTopic", 1);
	mEmotion_pub		= mNodeHandle.advertise<std_msgs::UInt8>("/cmd_emotion", 1);

	mDisableForward	 	= false;
	mDisableBackward 	= false;

	for (int i = 0; i < SensorFeedback::SENSOR_COUNT; i++)
		mSensorData[i] = 0;

	ROS_INFO("SafeKeeper initialised");
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "SafeKeeper");

	SafeKeeper safeKeeper;
	safeKeeper.init();

	int sleep_rate;
	safeKeeper.getNodeHandle()->param<int>("node_sleep_rate", sleep_rate, 50);
	ros::Rate sleep(sleep_rate);

	while (ros::ok())
	{
		sleep.sleep();
		ros::spinOnce();
	}
	return 0;
}
