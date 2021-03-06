/*
 * SoundPlayer.h
 *
 *  Created on: 2012-01-24
 *      Author: wilson
 */

#ifndef SOUNDPLAYER_H_
#define SOUNDPLAYER_H_

#include <ros/ros.h>
#include <std_msgs/UInt8.h>
#include <sound_play/sound_play.h>

enum Emotions
{
	NEUTRAL = 0,
	HAPPY,
	SAD,
	SURPRISED,
	ERROR
};

class SoundPlayer
{
protected:
	ros::NodeHandle mNodeHandle;	/// ROS node handle
	ros::Subscriber mSoundSub;		/// Listens	to commands and plays the corresponding wav file

	sound_play::SoundClient mSoundClient;
public:
	/// Constructor
	SoundPlayer() : mNodeHandle("") {};

	/// Destructor
	/** shut down node handle, stop 'run' thread */
	~SoundPlayer()
	{
		mNodeHandle.shutdown();
	};

	void init();
	void playCB(const std_msgs::UInt8& msg);
	bool playSound(std::string Path);

	inline ros::NodeHandle* getNodeHandle() { return &mNodeHandle; };
};

#endif /* SOUNDPLAYER_H_ */
