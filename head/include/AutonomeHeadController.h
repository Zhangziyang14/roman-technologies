#ifndef __AUTONOMECONTROLLER_H
#define __AUTONOMECONTROLLER_H

#include <ros/ros.h>
#include <std_msgs/UInt8.h>
#include <iostream>
#include "Emotion.h"
#include <head/rgb.h>

enum Emotions
{
  NEUTRAL = 0,
  HAPPY,
  SAD,
  SURPRISED,
  ERROR  
};

class autonomeHeadController
{
protected:
	ros::NodeHandle mNodeHandle;

	ros:Publisher mRGB_pub;

	Emotion mNeutral;
	Emotion mHappy;
	Emotion mSad;
	Emotion mSurprised;
	Emotion mError;


	


public:
	//Constructor
	autonomeHeadController() : mNodeHandle("") , neutral(255,255,255) , happy(255,255,0) , sad(255,0,255) , surprised(0,120,255) , error(0,255,255) { }

	/// Destructor
	~autonomeHeadController()
	{

		mNodeHandle.shutdown();
	}



	void expressEmotion(const std_msgs::UInt8 &msg);



};

#endif
