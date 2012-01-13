#ifndef __AUTONOMECONTROLLER_H
#define __AUTONOMECONTROLLER_H

#include <ros/ros.h>
#include <std_msgs/ColorRGBA.h>
#include <std_msgs/UInt8.h>
#include <head/eyebrows.h>
#include <head/panTilt.h>
#include <geometry_msgs/Pose.h>
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

	void headCommandCB(const geometry_msgs::Pose &msg);
	void expressEmotionCB(const std_msgs::UInt8 &msg);
	void init();



};

#endif
