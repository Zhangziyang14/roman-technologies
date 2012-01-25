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
	constructEmotion(mNeutral,   0, 0, 0,   0, 0, 0,   0,   90, 90, 79,   0, 0, 0);
	constructEmotion(mHappy,   0, 255, 0,   0, 255, 0,   0,   90, 90, 79,   0, 0, 0);
	constructEmotion(mSad,   0, 150, 255,   0, 150, 255,   0,   60, 120, 79,   0, 0, 0);
	constructEmotion(mSurprised,   250, 40, 0,   250, 40, 0,   0,   60, 120, 50,   0, 0, 0);
	constructEmotion(mError,   255, 0, 0,   255, 0, 0,   0,   90, 90, 79,   0, 0, 0);

	ROS_INFO("AutonomeHeadController initialised.");
}

/**
 * Sets the expression to the given emotion
 */
void AutonomeHeadController::setExpression(head::Emotion emotion)
{
	mRGB_pub.publish(emotion.rgb);
	mEyebrows_pub.publish(emotion.eyebrows);
	/*std_msgs::ColorRGBA rgb_msg;
	head::eyebrows eyebrows_msg;

	rgb_msg.r = emotion.red();
	rgb_msg.g = emotion.green();
	rgb_msg.b = emotion.blue();
	rgb_msg.a = 0;

	eyebrows_msg.lift = emotion.liftEyebrow();
	eyebrows_msg.left = emotion.leftEyebrow();
	eyebrows_msg.right = emotion.rightEyebrow();

	mRGB_pub.publish(rgb_msg);
	if (emotion.leftEyebrowTime() || emotion.rightEyebrowTime() || emotion.liftEyebrowTime())
	{
		uint64_t time = ros::Time::now().toNSec();
		uint64_t time_passed = 0;
		ros::Rate refresh_rate(10);
		while (ros::Time::now().toNSec() - time < emotion.leftEyebrowTime() ||
				ros::Time::now().toNSec() - time < emotion.rightEyebrowTime() ||
				ros::Time::now().toNSec() - time < emotion.liftEyebrowTime())
		{
			if (emotion.leftEyebrowTime())
				eyebrows_msg.left = mCurrentLeftAngle + (mCurrentLeftAngle - emotion.leftEyebrow()) *
					std::min(1.0, double(time_passed) / double(emotion.leftEyebrowTime()));
			if (emotion.rightEyebrowTime())
				eyebrows_msg.right = mCurrentRightAngle + (mCurrentRightAngle - emotion.rightEyebrow()) *
					std::min(1.0, double(time_passed) / double(emotion.rightEyebrowTime()));
			if (emotion.liftEyebrowTime())
				eyebrows_msg.lift = mCurrentLiftAngle + (mCurrentLiftAngle - emotion.liftEyebrow()) *
					std::min(1.0, double(time_passed) / double(emotion.liftEyebrowTime()));

			refresh_rate.sleep();
			time_passed = ros::Time::now().toNSec() - time;

			mEyebrows_pub.publish(eyebrows_msg);
		}
	}

	mCurrentLeftAngle = eyebrows_msg.left;
	mCurrentRightAngle = eyebrows_msg.right;
	mCurrentLiftAngle = eyebrows_msg.lift;*/
}

/**
 * Sets the expression as given by emotionTopic
 */
void AutonomeHeadController::expressEmotionCB(const std_msgs::UInt8 &msg)
{
	mCurrentEmotion = msg.data;

	switch(mCurrentEmotion)
	{
	case NEUTRAL: 	setExpression(mNeutral); break;
	case HAPPY: 	setExpression(mHappy); break;
	case SAD: 		setExpression(mSad); break;
	case SURPRISED: setExpression(mSurprised); break;
	case ERROR: 	setExpression(mError); break;
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
	mRGB_pub = mNodeHandle.advertise<head::RGB>("/rgbTopic", 1);
	mEyebrows_pub = mNodeHandle.advertise<head::Eyebrows>("/eyebrowTopic", 1);
	mHead_movement_pub = mNodeHandle.advertise<head::PitchYaw>("/headPositionTopic", 1);

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

	AutonomeHeadController *headController = new AutonomeHeadController();
	headController->init();

	while (ros::ok())
	{
		ros::spin();
	}


	return 0;
}

