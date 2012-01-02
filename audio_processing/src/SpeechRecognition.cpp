/*
 * SpeechRecognition.cpp
 *
 *  Created on: 2011-12-30
 *      Author: wilson
 */

#include <audio_processing/SpeechRecognition.h>

using namespace std;
/*
 * Listens to incoming speech, filters the meaning and maps commands to actions
 */
void SpeechRecognition::speechCB(const std_msgs::String& msg)
{
	audio_processing::speech processed_msg;
	uint8_t arousal = 0;

	//Listen to calls or commands from user, can be replaced by more efficient and more extensive checks
	if(msg.data.find("wake") != string::npos)
		processed_msg.command = "wake up";

	else if(msg.data.find("juice") != string::npos)
			processed_msg.command = "juice";

	else if(msg.data.find("sleep") != string::npos)
		processed_msg.command = "sleep";
	else
		processed_msg.command = "";

	//Listen to feedback, can be replaced by lists like anew
	if(msg.data.find("good") != string::npos)
		arousal = 10;

	if(msg.data.find("thanks") != string::npos)
		arousal = 10;

	if(msg.data.find("thank you") != string::npos)
		arousal = 10;

	if(msg.data.find("bad") != string::npos)
		arousal = 0;

	//TODO: When will Nero display novelty?

	processed_msg.arousal = arousal;

	mProcessedSpeechPublisher.publish(processed_msg);
}

void SpeechRecognition::init()
{
	//Initialise subscribers
	mSpeechSubscriber = mNodeHandle.subscribe("/recognizer/output", 1, &SpeechRecognition::speechCB, this);

	//Initialise publishers
	mProcessedSpeechPublisher = mNodeHandle.advertise<audio_processing::speech>("/processedSpeechTopic", 1);

	ROS_INFO("Initialised SpeechRecognition");
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "SpeechRecognition");
	SpeechRecognition speechRecognition;
	speechRecognition.init();

	ros::spin();
}
