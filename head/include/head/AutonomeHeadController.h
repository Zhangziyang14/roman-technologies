#ifndef __AUTONOMECONTROLLER_H
#define __AUTONOMECONTROLLER_H

#include <ros/ros.h>
#include <std_msgs/ColorRGBA.h>
#include <std_msgs/UInt8.h>
#include <head/eyebrows.h>
#include <head/PitchYaw.h>
#include <iostream>
#include "Emotion.h"


enum Emotions
{
	NEUTRAL = 0,
	HAPPY,
	SAD,
	SURPRISED,
	ERROR
};

class AutonomeHeadController
{
protected:
	ros::NodeHandle mNodeHandle;

	ros::Publisher mRGB_pub;
	ros::Publisher mHead_movement_pub;
	ros::Publisher mEyebrows_pub;

	ros::Subscriber mEmotion_sub;
	ros::Subscriber mCommand_sub;
	u_int8_t mCurrentEmotion;

	Emotion mNeutral;
	Emotion mHappy;
	Emotion mSad;
	Emotion mSurprised;
	Emotion mError;

public:
	//Constructor
	AutonomeHeadController() : mNodeHandle("") , mNeutral(0, 0, 0, 90, 90, 79) , mHappy(0, 255, 0, 90, 90, 79) , mSad(0, 150, 255, 60, 120, 79) , mSurprised(250, 40, 0, 60, 120, 50) , mError(255, 0, 0, 90, 90, 79) {
		ROS_INFO("AutonomeHeadController constructor");
	};

	/// Destructor
	~AutonomeHeadController()
	{

		mNodeHandle.shutdown();
	};

	void setExpression(Emotion emotion);
	void headCommandCB(const head::PitchYaw &msg);
	void expressEmotionCB(const std_msgs::UInt8 &msg);
	void init();



};

#endif
