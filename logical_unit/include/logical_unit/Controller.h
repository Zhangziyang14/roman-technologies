/*
 * Controller.h
 *
 *  Created on: 2011-11-13
 *      Author: wilson
 */

#ifndef CONTROLLER_H_
#define CONTROLLER_H_

#include <ros/ros.h>
#include <audio_processing/speech.h>
#include <map>

enum commandValue
{
	NOTHING = -1,
			WAKE_UP,
			JUICE,
			SLEEP,
};

enum Emotions
{
	NEUTRAL = 0,
			HAPPY,
			SAD,
			SURPRISED,
};

class Controller
{
private:
	ros::NodeHandle mNodeHandle;
	//ros::Publisher mGripperPublisher;
	//ros::Publisher mArmPublisher;
	//ros::Publisher mHeadPublisher;

	//ros::Subscriber mKinectSubscriber;
	ros::Subscriber mSpeechSubscriber;		/// Listens to speech processed by SpeechRecognition node

	bool mWakeUp;							/// Keeps track of whether Nero is listening to commands or not
	std::string mSpeech;					/// Keeps track of what is said by the user
	uint8_t mArousal;						/// Keeps track of the arousal contained in the speech of user
	Emotions mEmotionalState;				/// Keeps track of Nero's current emotion

public:
	Controller() : mNodeHandle(""){}

	virtual ~Controller(){ mNodeHandle.shutdown();}

	void init();
	void speechCB(const audio_processing::speech& msg);
};

#endif /* CONTROLLER_H_ */
