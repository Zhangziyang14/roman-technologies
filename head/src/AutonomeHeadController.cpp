#include "head/AutonomeHeadController.h"

void constructEmotion(head::Emotion &emotion, uint8_t min_r, uint8_t min_g, uint8_t min_b, uint8_t max_r, uint8_t max_g, uint8_t max_b, uint32_t breath_time, int left_eyebrow, int right_eyebrow, int lift, uint32_t left_time, uint32_t right_time, uint32_t lift_time)
{
	emotion.rgb.min_r = min_r;
	emotion.rgb.min_g = min_g;
	emotion.rgb.min_b = min_b;
	emotion.rgb.max_r = max_r;
	emotion.rgb.max_g = max_g;
	emotion.rgb.max_b = max_b;
	emotion.rgb.breath_time = breath_time;
	emotion.eyebrows.left = left_eyebrow;
	emotion.eyebrows.left_time = left_time;
	emotion.eyebrows.right = right_eyebrow;
	emotion.eyebrows.right_time = right_time;
	emotion.eyebrows.lift = lift;
	emotion.eyebrows.lift_time = lift_time;
}

AutonomeHeadController::AutonomeHeadController(): mNodeHandle("")
{
	constructEmotion(mNeutral,   255, 255, 255,   150, 150, 150,   3000,   90, 90, 120,   0, 0, 0);
	constructEmotion(mHappy,   50, 250, 30,   255, 150, 30,   1500,   90, 90, 120,   0, 0, 0);
	constructEmotion(mSad,   50, 50, 255,   150, 110, 255,   4000,   60, 120, 120,   0, 0, 0);
	constructEmotion(mSurprised,   255, 120, 20,   255, 100, 0,   1000,   60, 120, 140,   0, 0, 0);
	constructEmotion(mError,   50, 0, 0,   255, 0, 0,   1000,   90, 90, 120,   0, 0, 0);
	constructEmotion(mSleep,   20, 20, 20,   50, 50, 50,   5000,   90, 90, 120,   0, 0, 0);

	ROS_INFO("AutonomeHeadController initialised.");
}

/**
 * Sets the expression to the given emotion
 */
void AutonomeHeadController::setExpression(head::Emotion emotion)
{
	mRGB_pub.publish(emotion.rgb);
	mEyebrows_pub.publish(emotion.eyebrows);

	std_msgs::UInt8 sound_msg;
	sound_msg.data = mCurrentEmotion;

	mSounds_pub.publish(sound_msg);
}

/**
 * Sets the expression as given by emotionTopic
 */
void AutonomeHeadController::expressEmotionCB(const std_msgs::UInt8 &msg)
{
	mCurrentEmotion = msg.data;

	switch(mCurrentEmotion)
	{
	case head::Emotion::NEUTRAL:	setExpression(mNeutral); break;
	case head::Emotion::HAPPY:		setExpression(mHappy); break;
	case head::Emotion::SAD: 		setExpression(mSad); break;
	case head::Emotion::SURPRISED:	setExpression(mSurprised); break;
	case head::Emotion::ERROR: 		setExpression(mError); break;
	case head::Emotion::SLEEP:		setExpression(mSleep); break;

	default:
		setExpression(mNeutral);
	}
}

/**
 * Initialise AutonomeHeadController
 */
void AutonomeHeadController::init()
{
	// initialise subscribers
	//mBumperFeedback_sub = mNodeHandle.subscribe("/bumperFeedbackTopic", 10, &SafeKeeper::bumperFeedbackCB, this);
	mEmotion_sub = mNodeHandle.subscribe("/cmd_emotion",1, &AutonomeHeadController::expressEmotionCB, this);
	mCommand_sub = mNodeHandle.subscribe("/cmd_head_position",1, &AutonomeHeadController::headCommandCB, this);

	// initialise publishers
	mRGB_pub 			= mNodeHandle.advertise<head::RGB>("/rgbTopic", 1, true);
	mEyebrows_pub 		= mNodeHandle.advertise<head::Eyebrows>("/eyebrowTopic", 1, true);
	mSounds_pub			= mNodeHandle.advertise<std_msgs::UInt8>("/cmd_sound", 1, true);
	mHead_movement_pub 	= mNodeHandle.advertise<head::PitchYaw>("/headPositionTopic", 1, true);

	head::PitchYaw msg;
	msg.pitch = 0.0;
	msg.yaw = 0.0;
	mHead_movement_pub.publish(msg);
	setExpression(mNeutral);

	ROS_INFO("AutonomeHeadController initialised");
}

/**
 * Listens to head commands and streams them to HeadMotorHandler
 */
void AutonomeHeadController::headCommandCB(const head::PitchYaw &msg)
{
	mHead_movement_pub.publish(msg);
}

int main(int argc, char **argv)
{

	ros::init(argc, argv, "AutonomeHeadController");

	AutonomeHeadController headController;
	headController.init();

	int sleep_rate;
	headController.getNodeHandle()->param<int>("node_sleep_rate", sleep_rate, 50);
	ros::Rate sleep(sleep_rate);

	while (ros::ok())
	{
		sleep.sleep();
		ros::spinOnce();
	}


	return 0;
}

