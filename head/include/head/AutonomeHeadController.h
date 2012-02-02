#ifndef __AUTONOMECONTROLLER_H
#define __AUTONOMECONTROLLER_H

#include <ros/ros.h>
#include <std_msgs/UInt8.h>
#include <head/PitchYaw.h>
#include <iostream>
#include "head/Emotion.h"
#include "head/RGB.h"
#include "head/Eyebrows.h"
#include <stdint.h>

class AutonomeHeadController
{
protected:
	ros::NodeHandle mNodeHandle;

	ros::Publisher mHead_movement_pub;
	ros::Publisher mSounds_pub;
	ros::Publisher mRGB_pub;
	ros::Publisher mEyebrows_pub;

	ros::Subscriber mEmotion_sub;
	ros::Subscriber mCommand_sub;
	uint8_t mCurrentEmotion;
	double mReturnNeutralTime;

	head::Emotion mNeutral;
	head::Emotion mHappy;
	head::Emotion mSad;
	head::Emotion mSurprised;
	head::Emotion mError;
	head::Emotion mSleep;

public:
	//Constructor
	AutonomeHeadController();

	/// Destructor
	~AutonomeHeadController()
	{

		mNodeHandle.shutdown();
	};

	void setExpression(head::Emotion emotion);
	void headCommandCB(const head::PitchYaw &msg);
	void expressEmotionCB(const std_msgs::UInt8 &msg);
	void init();
	void update();

	inline ros::NodeHandle* getNodeHandle() { return &mNodeHandle; };
};

#endif
