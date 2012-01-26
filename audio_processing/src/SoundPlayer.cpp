/*
 * SoundPlayer.cpp
 *
 *  Created on: 2012-01-24
 *      Author: wilson
 */

#include <audio_processing/SoundPlayer.h>

/// Maps incoming Emotion to a file path for playback
static std::map<u_int8_t, std::string> emotionToPath;

bool SoundPlayer::playSound(std::string Path)
{
	if (!mBuffer.LoadFromFile(Path.c_str()))
		return false;

	// Create a sound instance and play it
	sf::Sound Sound(mBuffer);
	Sound.Play();
	sf::Sleep(0.4f);

	// Loop while the sound is playing
	while (Sound.GetStatus() == sf::Sound::Playing)
	{
		// Display the playing position
		std::cout << "\rPlaying... " << std::fixed << std::setprecision(2) << Sound.GetPlayingOffset() << " sec";

		// Leave some CPU time for other threads
		sf::Sleep(0.1f);
	}

	return true;
}

void SoundPlayer::playCB(const std_msgs::UInt8& msg)
{
	if(emotionToPath.find(msg.data) == emotionToPath.end())
	{
		ROS_WARN("No wav file for this emotion");
		return;
	}

	if(playSound(emotionToPath[msg.data]))
		ROS_INFO("Played sound successfully");
	else
		ROS_ERROR("Could not find wav file");
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
