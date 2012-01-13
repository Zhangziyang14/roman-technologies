#include "head/AutonomeHeadController.h"

std_msgs::ColorRGBA rgb;

void AutonomeHeadController::expressEmotionCB(const std_msgs::UInt8 &msg)
{

	switch(msg.data)
	{
	case NEUTRAL:
	{
		rgb.r = mNeutral.red();
		rgb.g = mNeutral.green();
		rgb.b = mNeutral.blue();

	}
	break;

	case HAPPY:
	{
		rgb.r = mHappy.red();
		rgb.g = mHappy.green();
		rgb.b = mHappy.blue();
	}
	break;

	case SAD:
	{
		rgb.r = mSad.red();
		rgb.g = mSad.green();
		rgb.b = mSad.blue();
	}
	break;

	case SURPRISED:
	{
		rgb.r = mSurprised.red();
		rgb.g = mSurprised.green();
		rgb.b = mSurprised.blue();
	}
	break;

	case ERROR:
	{
		rgb.r = mError.red();
		rgb.g = mError.green();
		rgb.b = mError.blue();
	}
	break;

	default:
	{
		rgb.r = mNeutral.red();
		rgb.g = mNeutral.green();
		rgb.b = mNeutral.blue();
	}



	}
	ROS_INFO("Publish Colors: %f, %f, %f", rgb.r, rgb.g, rgb. g);
	mRGB_pub.publish(rgb);


}
void AutonomeHeadController::init()
{
	// initialise subscribers
	//mBumperFeedback_sub = mNodeHandle.subscribe("/bumperFeedbackTopic", 10, &SafeKeeper::bumperFeedbackCB, this);
	mEmotion_sub = mNodeHandle.subscribe("/emotionTopic",1, &AutonomeHeadController::expressEmotionCB, this);
	mCommand_sub = mNodeHandle.subscribe("/cmd_head_position",1, &AutonomeHeadController::headCommandCB, this);

	// initialise publishers
	mRGB_pub = mNodeHandle.advertise<std_msgs::ColorRGBA>("/rgbTopic", 1);
	mEyebrows_pub = mNodeHandle.advertise<head::eyebrows>("/eyebrowsTopic", 1);
	mHead_movement_pub = mNodeHandle.advertise<geometry_msgs::Pose>("/headPositionTopic", 1);

	ROS_INFO("AutonomeHeadController initialised");

	rgb.a = 0;
}

/**
 * Listens to head commands and streams them to HeadMotorHandler
 */
void AutonomeHeadController::headCommandCB(const geometry_msgs::Pose &msg)
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

