#include "head/AutonomeHeadController.h"

void constructEmotion(nero_msgs::Emotion &emotion, uint8_t min_r, uint8_t min_g, uint8_t min_b, uint8_t max_r, uint8_t max_g, uint8_t max_b, uint32_t breath_time, int left_eyebrow, int right_eyebrow, int lift, uint32_t left_time, uint32_t right_time, uint32_t lift_time, uint32_t neutral_time)
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
	emotion.time = neutral_time;
}

AutonomeHeadController::AutonomeHeadController(): mNodeHandle("")
{
	constructEmotion(mNeutral,		255, 255, 255,	150, 150, 150,	3000,	90, 90, 120,	0, 0, 0,		0);
	constructEmotion(mHappy,		255, 255, 150,	255, 150, 30,	800,	90, 90, 120,	0, 0, 0,		2500);
	constructEmotion(mSad,			50, 50, 255,	150, 110, 255,	4000,	60, 120, 120,	500, 500, 0,	2500);
	constructEmotion(mSurprised,	255, 255, 255, 	150, 150,150,	250, 	90, 90, 140, 	0, 0, 0,		2500);
	constructEmotion(mError,		50, 0, 0,		255, 0, 0,		1000,	120, 60, 120,	0, 0, 0,		2500);
	constructEmotion(mSleep,		20, 20, 20,		50, 50, 50,		5000,	90, 90, 120,	0, 0, 0,		0);

	mReturnNeutralTime = 0.0;
}

/**
 * Sets the expression to the given emotion
 */
void AutonomeHeadController::setExpression(nero_msgs::Emotion emotion, int soundId)
{
	mRGB_pub.publish(emotion.rgb);
	mEyebrows_pub.publish(emotion.eyebrows);

	if (soundId >= 0)
	{
		std_msgs::UInt8 sound_msg;
		sound_msg.data = uint8_t(soundId);

		mSounds_pub.publish(sound_msg);
	}

	mReturnNeutralTime = emotion.time ? ros::Time::now().toSec() + double(emotion.time) / 1000.0 : 0.0;
}

/**
 * Sets the expression as given by emotionTopic
 */
void AutonomeHeadController::expressEmotionCB(const std_msgs::UInt8 &msg)
{
	switch(msg.data)
	{
	case nero_msgs::Emotion::NEUTRAL:	setExpression(mNeutral, msg.data); break;
	case nero_msgs::Emotion::HAPPY:		setExpression(mHappy, msg.data); break;
	case nero_msgs::Emotion::SAD: 		setExpression(mSad, msg.data); break;
	case nero_msgs::Emotion::SURPRISED:	setExpression(mSurprised, msg.data); break;
	case nero_msgs::Emotion::ERROR: 	setExpression(mError, msg.data); break;
	case nero_msgs::Emotion::SLEEP:		setExpression(mSleep, msg.data); break;

	default:
		setExpression(mNeutral);
		break;
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
	mTrackSub	 = mNodeHandle.subscribe("/Head/follow_point",1, &AutonomeHeadController::trackCb, this);

	// initialise publishers
	mRGB_pub 			= mNodeHandle.advertise<nero_msgs::RGB>("/rgbTopic", 1, true);
	mEyebrows_pub 		= mNodeHandle.advertise<nero_msgs::Eyebrows>("/eyebrowTopic", 1, true);
	mSounds_pub			= mNodeHandle.advertise<std_msgs::UInt8>("/cmd_sound", 1, true);
	mHead_movement_pub 	= mNodeHandle.advertise<nero_msgs::PitchYaw>("/headPositionTopic", 1, true);

	// set the head to sleep angle
	nero_msgs::PitchYaw msg;
	msg.pitch = 0.8;
	msg.yaw = 0.0;
	mHead_movement_pub.publish(msg);
	setExpression(mSleep);

	ROS_INFO("AutonomeHeadController initialised");
}

/**
 * Listens to head commands and streams them to HeadMotorHandler
 */
void AutonomeHeadController::headCommandCB(const nero_msgs::PitchYaw &msg)
{
	mHead_movement_pub.publish(msg);
}

void AutonomeHeadController::trackCb(const geometry_msgs::PointStamped &msg)
{
	if (msg.header.frame_id != "/head_frame")
	{
		ROS_ERROR("[AutonomeHeadController] Track point not in /head_frame!");
		return;
	}

    double pitch = -atan(msg.point.z / msg.point.y);
    double yaw = -atan(msg.point.x / msg.point.y);

    nero_msgs::PitchYaw headmsg;
    headmsg.pitch = pitch;
    headmsg.yaw = yaw;
    mHead_movement_pub.publish(headmsg);
}

void AutonomeHeadController::update()
{
	if (mReturnNeutralTime && ros::Time::now().toSec() > mReturnNeutralTime)
	{
		std_msgs::UInt8 msg;
		msg.data = nero_msgs::Emotion::NEUTRAL;
		expressEmotionCB(msg);
	}
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
		headController.update();
		sleep.sleep();
		ros::spinOnce();
	}

	return 0;
}
