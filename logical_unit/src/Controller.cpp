/*
 * Controller.cpp
 *
 *  Created on: 2011-11-13
 *      Author: wilson
 */

#include "logical_unit/Controller.h"

static std::map<std::string, commandValue> stringToValue; /// Maps incoming string to a commandValue for use in switch-case in main function

void Controller::speechCB(const audio_processing::speech& msg)
{
	mSpeech		 = msg.command;
	mArousal 	 = msg.arousal;

	ROS_INFO("String: %s, Value: %d", mSpeech.c_str(), stringToValue[mSpeech]);
	switch(stringToValue[mSpeech])
	{
	case WAKE_UP:
		if(!mWakeUp)
		{
			mWakeUp = true;
			ROS_INFO("Heard user...");
			//Start initiating actions to show that the robot has heard the user
		}
		else
			ROS_INFO("I'm already awake");
		break;

	case JUICE:
		if(mWakeUp)
			ROS_INFO("Getting juice...");
		//Start initiating actions to get the juice
		else
			ROS_INFO("Don't disturb me in my sleep...");
		break;

	case SLEEP:
		if(mWakeUp)
		{
			//Go to sleep
			mWakeUp = false;
			ROS_INFO("Going to sleep...");
		}
		else
			ROS_INFO("I'm already sleeping...");
		break;

	case NOTHING:
		ROS_INFO("No commands");
	default:
		break;
	}
}
//TODO Decide how to score recognized arousal/emotions

void Controller::init()
{
	//initialise map
	stringToValue[""]		 = NOTHING;
	stringToValue["wake up"] = WAKE_UP;
	stringToValue["juice"]   = JUICE;
	stringToValue["sleep"]	 = SLEEP;

	//initialise subscribers
	mSpeechSubscriber = mNodeHandle.subscribe("processedSpeechTopic", 1, &Controller::speechCB, this);

	//initialise publishers
	ROS_INFO("Initialised Controller");
	mWakeUp = false;
}

int main(int argc, char **argv)
{
	// init ros and pathfinder
	ros::init(argc, argv, "controller");
	Controller controller;

	char *path = NULL;
	if (argc == 2)
		path = argv[1];

	controller.init();

	ros::spin();
}
