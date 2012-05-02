/*
 * SoundPlayer.cpp
 *
 *  Created on: 2012-01-24
 *      Author: wilson
 */

#include <audio_processing/SoundPlayer.h>

/// Maps incoming Emotion to a file path for playback
static std::map<u_int8_t, std::string> emotionToPath;

void SoundPlayer::playCB(const std_msgs::UInt8& msg)
{
	if(emotionToPath.find(msg.data) == emotionToPath.end())
	{
		ROS_WARN("No wav file for this emotion");
		return;
	}

	mSoundClient.playWave(emotionToPath[msg.data]);

	ROS_INFO("Played sound");
}

/**
 * Initialise SoundPlayer
 */
void SoundPlayer::init()
{
	//Initialise Subscribers
	mSoundSub = mNodeHandle.subscribe("/cmd_sound", 1, &SoundPlayer::playCB, this);

	ROS_INFO("Initialising complete");
}


int main( int argc, char* argv[])
{
	ros::init(argc, argv, "SoundPlayer");

	if (argc != 4)
	{
		ROS_ERROR("Invalid input arguments.");
		return 0;
	}

	SoundPlayer soundPlayer;
	soundPlayer.init();

	emotionToPath[HAPPY] 	 = argv[1];
	emotionToPath[SAD]   	 = argv[2];
	emotionToPath[SURPRISED] = argv[3];

	int sleep_rate;
	soundPlayer.getNodeHandle()->param<int>("node_sleep_rate", sleep_rate, 50);
	ros::Rate sleep(sleep_rate);

	while (ros::ok())
	{
		sleep.sleep();
		ros::spinOnce();
	}

	return EXIT_SUCCESS;
}
