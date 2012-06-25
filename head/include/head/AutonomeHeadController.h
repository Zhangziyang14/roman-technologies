#ifndef __AUTONOMECONTROLLER_H
#define __AUTONOMECONTROLLER_H

#include <ros/ros.h>
#include <std_msgs/UInt8.h>
#include <nero_msgs/PitchYaw.h>
#include <iostream>
#include "nero_msgs/Emotion.h"
#include "nero_msgs/RGB.h"
#include "nero_msgs/Eyebrows.h"
#include <stdint.h>
#include "geometry_msgs/PointStamped.h"

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
	ros::Subscriber mTrackSub;
	double mReturnNeutralTime;

	nero_msgs::Emotion mNeutral;
	nero_msgs::Emotion mHappy;
	nero_msgs::Emotion mSad;
	nero_msgs::Emotion mSurprised;
	nero_msgs::Emotion mError;
	nero_msgs::Emotion mSleep;

public:
	//Constructor
	AutonomeHeadController();

	/// Destructor
	~AutonomeHeadController()
	{

		mNodeHandle.shutdown();
	};

	void setExpression(nero_msgs::Emotion emotion, int soundId = -1);
	void headCommandCB(const nero_msgs::PitchYaw &msg);
	void expressEmotionCB(const std_msgs::UInt8 &msg);
	void trackCb(const geometry_msgs::PointStamped &msg);
	void init();
	void update();

	inline ros::NodeHandle* getNodeHandle() { return &mNodeHandle; };
};

#endif
