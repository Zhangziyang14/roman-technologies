/*
 * PS3Controller.cpp
 *
 *  Created on: Aug 17, 2012
 *      Author: hans
 */

#include "nero_tools/PS3Controller.h"

bool pressed(std::vector<int> previousButtons, const sensor_msgs::Joy &joy, PS3Key key)
{
	return previousButtons[key] == 0 && joy.buttons[key] == 1;
}

PS3Controller::PS3Controller() :
	mPS3ModeCount(0),
	mPS3ModeIndex(0),
	mPS3Active(false)
{
	mPS3Sub 		= mNodeHandle.subscribe("/joy", 5, &PS3Controller::controllerCb, this);
	mBatteryLogSub 	= mNodeHandle.subscribe("/batteryLogTopic", 5, &PS3Controller::batteryLogCb, this);
	mPS3FeedbackPub	= mNodeHandle.advertise<sensor_msgs::JoyFeedbackArray>("/joy/set_feedback", 1);

	memset(mPS3Modes, 0, sizeof(ControllerMode) * MAX_MODES);

	ROS_INFO("[PS3Controller] Initialized.");
}

void PS3Controller::controllerCb(const sensor_msgs::Joy& joy)
{
	if (mPreviousButtons.empty())
		mPreviousButtons = joy.buttons; // first callback trigger
	if (mPreviousAxes.empty())
		mPreviousAxes = joy.axes;

	// toggle modes
	if (mPS3ModeCount > 1)
	{
		if (mPreviousButtons[PS3_LEFT_STICK] && pressed(mPreviousButtons, joy, PS3_RIGHT_STICK)) // shift up one mode
		{
			if (getCurrentMode())
				getCurrentMode()->onDeactivate();
			setLED(mPS3ModeIndex, false);
			mPS3ModeIndex = (mPS3ModeIndex + 1) % mPS3ModeCount;
			setLED(mPS3ModeIndex, true);
			if (getCurrentMode())
				getCurrentMode()->onActivate();
		}
		else if (pressed(mPreviousButtons, joy, PS3_LEFT_STICK) && mPreviousButtons[PS3_RIGHT_STICK]) // shift down one mode
		{
			if (getCurrentMode())
				getCurrentMode()->onDeactivate();
			setLED(mPS3ModeIndex, false);
			mPS3ModeIndex = (mPS3ModeIndex == 0 ? mPS3ModeCount : mPS3ModeIndex) - 1;
			setLED(mPS3ModeIndex, true);
			if (getCurrentMode())
				getCurrentMode()->onActivate();
		}
	}

	if (getCurrentMode() == NULL)
	{
		ROS_ERROR("PS3Mode with index %d does not exist!", mPS3ModeIndex);
		mPreviousButtons = joy.buttons;
		mPreviousAxes = joy.axes;
		return;
	}

	getCurrentMode()->handleController(mPreviousButtons, mPreviousAxes, joy);
	mPreviousButtons = joy.buttons;
	mPreviousAxes = joy.axes;
}

void PS3Controller::batteryLogCb(const std_msgs::UInt16& log)
{
	if (log.data <= BATTERY_EMPTY)
		setRumble(1.f);
}

void PS3Controller::setLED(uint8_t led, bool on)
{
	if (led >= LED_COUNT)
	{
		ROS_ERROR("[PS3Controller] LED id is out of bounds!");
		return;
	}

	sensor_msgs::JoyFeedbackArray msg;

	sensor_msgs::JoyFeedback feedback;
	feedback.type = sensor_msgs::JoyFeedback::TYPE_LED;
	feedback.id = led;
	feedback.intensity = on ? 1.f : 0.f;
	msg.array.push_back(feedback);

	mPS3FeedbackPub.publish(msg);
}

void PS3Controller::setRumble(float intensity)
{
	sensor_msgs::JoyFeedbackArray msg;

	sensor_msgs::JoyFeedback feedback;
	feedback.type = sensor_msgs::JoyFeedback::TYPE_RUMBLE;
	feedback.intensity = intensity;
	msg.array.push_back(feedback);

	mPS3FeedbackPub.publish(msg);
}

void PS3Controller::addMode(ControllerMode *mode)
{
	if (mPS3ModeCount >= MAX_MODES)
	{
		ROS_ERROR("[PS3Controller] Max mode count reached!");
		return;
	}

	mPS3Modes[mPS3ModeCount++] = mode;
}

void PS3Controller::spin()
{
	while (ros::ok())
	{
		if (mPS3Sub.getNumPublishers())
		{
			if (mPS3Active == false && getCurrentMode())
				getCurrentMode()->onActivate();
		}
		else if (mPS3Active && getCurrentMode())
			getCurrentMode()->onDeactivate();

		mPS3Active = mPS3Sub.getNumPublishers() != 0;

		if (getCurrentMode())
			getCurrentMode()->runOnce();
		ros::spinOnce();
	}
}

int main( int argc, char* argv[] )
{
	ros::init(argc, argv, "PS3Controller");

	PS3Controller ps3Controller;
	ps3Controller.addMode(new DriveMode(ps3Controller.getNodeHandle()));
	ps3Controller.addMode(new ArmMode(ps3Controller.getNodeHandle()));
	ps3Controller.addMode(new HeadMode(ps3Controller.getNodeHandle()));
	ps3Controller.spin();
}
