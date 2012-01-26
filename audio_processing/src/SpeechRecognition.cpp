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
	// Arousal 1 is the neutral state
	// Arousal > 1 is the happy state
	// Arousal < 1 is the sad state

	audio_processing::speech processed_msg;
	int arousal = 1;

	//Listen to calls or commands from user, can be replaced by more efficient and more extensive checks
	if(msg.data.find("wake") != string::npos)
		processed_msg.command = "wake up";

	else if(msg.data.find("eva") != string::npos)
			processed_msg.command = "eva";

	else if(msg.data.find("juice") != string::npos)
			processed_msg.command = "juice";

	else if(msg.data.find("drink") != string::npos)
			processed_msg.command = "drink";

	else if(msg.data.find("coke") != string::npos)
			processed_msg.command = "coke";

	else if(msg.data.find("cola") != string::npos)
			processed_msg.command = "cola";

	else if(msg.data.find("sleep") != string::npos)
		processed_msg.command = "sleep";

	else if(msg.data.find("release") != string::npos)
		processed_msg.command = "release";

	else
		processed_msg.command = "";

	//Listen to feedback, can be replaced by lists like anew
	if(msg.data.find("good") != string::npos)
		arousal += 1;

	if(msg.data.find("thanks") != string::npos)
		arousal += 1;

	if(msg.data.find("great") != string::npos)
		arousal += 1;

	if(msg.data.find("yummy") != string::npos)
		arousal += 1;

	if(msg.data.find("delicious") != string::npos)
		arousal += 1;

	if(msg.data.find("nice") != string::npos)
		arousal += 1;

	if(msg.data.find("thank you") != string::npos)
		arousal += 1;

	if(msg.data.find("bad") != string::npos || msg.data.find("not") != string::npos)
		arousal = arousal >= 1 ? arousal *-1 : arousal;

	processed_msg.arousal = arousal;

	ROS_INFO("arousal %d msg %d", arousal, processed_msg.arousal);

	mProcessedSpeechPublisher.publish(processed_msg);
	arousal = 1;
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
