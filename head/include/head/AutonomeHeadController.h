#ifndef __AUTONOMECONTROLLER_H
#define __AUTONOMECONTROLLER_H

#include <ros/ros.h>
#include <std_msgs/ColorRGBA.h>
#include <std_msgs/UInt8.h>
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
	ros::Subscriber mEmotion_sub;

	Emotion mNeutral;
	Emotion mHappy;
	Emotion mSad;
	Emotion mSurprised;
	Emotion mError;


	


public:
	//Constructor
	AutonomeHeadController() : mNodeHandle("") , mNeutral(0,0,0) , mHappy(0,255,0) , mSad(0,150,255) , mSurprised(250,40,0) , mError(255,0,0) { 
		ROS_INFO("AutonomeHeadController constructor");
	}

	/// Destructor
	~AutonomeHeadController()
	{

		mNodeHandle.shutdown();
	}



	void expressEmotionCB(const std_msgs::UInt8 &msg);
	void init();



};

#endif
