/*
 * SpeechRecognition.h
 *
 *  Created on: 2011-12-30
 *      Author: wilson
 */

#ifndef SPEECHRECOGNITION_H_
#define SPEECHRECOGNITION_H_

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <iostream>
#include <string>
#include <nero_msgs/SpeechCommand.h>

class SpeechRecognition
{
protected:
	ros::NodeHandle mNodeHandle;				/// ROS node handle
	ros::Subscriber mSpeechSubscriber;			/// Listens to incoming speech
	ros::Publisher  mProcessedSpeechPublisher;	/// Publishes sentences to controller after processing

public:
	/// Constructor
	SpeechRecognition() : mNodeHandle("~") { }

	/// Destructor
	~SpeechRecognition()
	{
		mNodeHandle.shutdown();
	}

	/// Initialise node
	void init();
	void speechCB(const std_msgs::String& msg);

	inline ros::NodeHandle* getNodeHandle() { return &mNodeHandle; };
};

#endif /* SPEECHRECOGNITION_H_ */
