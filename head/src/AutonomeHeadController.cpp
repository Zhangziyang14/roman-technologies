#include "head/AutonomeHeadController.h"

/**
 * Sets the expression to the given emotion
 */
void AutonomeHeadController::setExpression(Emotion emotion)
{
	std_msgs::ColorRGBA rgb_msg;
	head::eyebrows eyebrows_msg;

	rgb_msg.r = emotion.red();
	rgb_msg.g = emotion.green();
	rgb_msg.b = emotion.blue();
	rgb_msg.a = 0;

	eyebrows_msg.lift = emotion.liftEyebrow();
	eyebrows_msg.left = emotion.leftEyebrow();
	eyebrows_msg.right = emotion.rightEyebrow();

	mRGB_pub.publish(rgb_msg);
	//mEyebrows_pub.publish(eyebrows_msg);
}

/**
 * Sets the expression as given by emotionTopic
 */
void AutonomeHeadController::expressEmotionCB(const std_msgs::UInt8 &msg)
{
	mCurrentEmotion = msg.data;

	switch(mCurrentEmotion)
	{
	case NEUTRAL:
	{
		setExpression(mNeutral);
	}
	break;

	case HAPPY:
	{
		setExpression(mHappy);
	}
	break;

	case SAD:
	{
		setExpression(mSad);
	}
	break;

	case SURPRISED:
	{
		setExpression(mSurprised);
	}
	break;

	case ERROR:
	{
		setExpression(mError);
	}
	break;

	default:
	{
		setExpression(mNeutral);
	}

	}
}

/**
 * Initialise AutonomeHeadController
 */
void AutonomeHeadController::init()
{
	// initialise subscribers
	//mBumperFeedback_sub = mNodeHandle.subscribe("/bumperFeedbackTopic", 10, &SafeKeeper::bumperFeedbackCB, this);
	mEmotion_sub = mNodeHandle.subscribe("/emotionTopic",1, &AutonomeHeadController::expressEmotionCB, this);
	mCommand_sub = mNodeHandle.subscribe("/cmd_head_position",1, &AutonomeHeadController::headCommandCB, this);

	// initialise publishers
	mRGB_pub = mNodeHandle.advertise<std_msgs::ColorRGBA>("/rgbTopic", 1);
	mEyebrows_pub = mNodeHandle.advertise<head::eyebrows>("/eyebrowTopic", 1);
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

