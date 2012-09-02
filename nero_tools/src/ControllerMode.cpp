/*
 * ControllerMode.cpp
 *
 *  Created on: Aug 17, 2012
 *      Author: hans
 */

#include "nero_tools/ControllerMode.h"

ControllerMode::ControllerMode(ros::NodeHandle *nodeHandle) :
		mAwake(true)
{
	mNodeHandle = nodeHandle;

	mEmotionPub 		= mNodeHandle->advertise<std_msgs::UInt8>("/cmd_emotion", 1);
	mSpeechCommandPub 	= mNodeHandle->advertise<nero_msgs::SpeechCommand>("/processedSpeechTopic", 1);
	mHeadPosPub			= mNodeHandle->advertise<nero_msgs::PitchYaw>("/cmd_head_position", 1);
	mPersonPosPub		= mNodeHandle->advertise<geometry_msgs::Point>("/PersonTracker/initial_point", 1);
}

bool ControllerMode::pressed(std::vector<int> previousButtons, const sensor_msgs::Joy &joy, PS3Key key)
{
	return previousButtons[key] == 0 && joy.buttons[key] == 1;
}

void ControllerMode::handleController(std::vector<int> previousButtons, std::vector<float> previousAxes, const sensor_msgs::Joy &joy)
{
	// Emotions
	if (pressed(previousButtons, joy, PS3_X))
		sendEmotion(nero_msgs::Emotion::HAPPY);
	if (pressed(previousButtons, joy, PS3_O))
		sendEmotion(nero_msgs::Emotion::SURPRISED);
	if (pressed(previousButtons, joy, PS3_S))
		sendEmotion(nero_msgs::Emotion::SAD);
	if (pressed(previousButtons, joy, PS3_T))
		sendEmotion(nero_msgs::Emotion::ERROR);

	// Wake up / sleep
	if (pressed(previousButtons, joy, PS3_START))
	{
		if (mAwake)
		{
			sendSpeechCommand("sleep");
			sendHeadPosition(HEAD_SLEEP_PITCH, HEAD_SLEEP_YAW);
		}
		else
		{
			sendSpeechCommand("wake up");
			sendHeadPosition(HEAD_AWAKE_PITCH, HEAD_AWAKE_YAW);
		}

		mAwake = !mAwake;
	}

	// fetch commands
	if (pressed(previousButtons, joy, PS3_LEFT))
		sendSpeechCommand("cola");
	if (pressed(previousButtons, joy, PS3_RIGHT))
		sendSpeechCommand("juice");

	// Person Follower
	if (pressed(previousButtons, joy, PS3_UP))
		sendPersonFollowing(320, 240);
	if (pressed(previousButtons, joy, PS3_DOWN))
		sendPersonFollowing(0, 0);
}

void ControllerMode::sendEmotion(uint8_t emotion)
{
	std_msgs::UInt8 msg;
	msg.data = emotion;
	mEmotionPub.publish(msg);

	ROS_INFO("[ControllerMode] Sending emotion (%d)", emotion);
}

void ControllerMode::sendSpeechCommand(std::string command)
{
	nero_msgs::SpeechCommand msg;
	msg.arousal = 0;
	msg.command = command;
	mSpeechCommandPub.publish(msg);

	ROS_INFO("[ControllerMode] Sending speech command (%s)", command.c_str());
}

void ControllerMode::sendHeadPosition(float pitch, float yaw)
{
	nero_msgs::PitchYaw msg;
	msg.pitch = pitch;
	msg.yaw = yaw;
	mHeadPosPub.publish(msg);

	ROS_INFO("[ControllerMode] Sending head position (%f, %f)", pitch, yaw);
}

void ControllerMode::sendPersonFollowing(int x, int y)
{
	geometry_msgs::Point msg;
	msg.x = x;
	msg.y = y;
	mPersonPosPub.publish(msg);

	ROS_INFO("[ControllerMode] Sending person position (%d, %d)", x, y);
}
