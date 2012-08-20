/*
 * PS3Controller.cpp
 *
 *  Created on: Aug 17, 2012
 *      Author: hans
 */

#include "ros/ros.h"
#include "nero_tools/PS3Controller.h"

PS3Controller::PS3Controller() :
	mPS3ModeCount(0),
	mPS3ModeIndex(0),
	mPS3Active(false)
{
	mPS3Sub = mNodeHandle.subscribe("/joy", 5, &PS3Controller::controllerCb, this);

	memset(mPS3Modes, 0, sizeof(ControllerMode) * MAX_MODES);

	ROS_INFO("[PS3Controller] Initialized.");
}

void PS3Controller::controllerCb(const sensor_msgs::Joy& joy)
{
	if (mPreviousJoy.buttons[PS3_LEFT_STICK] && joy.buttons[PS3_RIGHT_STICK]) // shift up one mode
	{
		if (getCurrentMode())
			getCurrentMode()->onDeactivate();
		mPS3ModeIndex = (mPS3ModeIndex + 1) % MAX_MODES;
		if (getCurrentMode())
			getCurrentMode()->onActivate();
	}
	else if (joy.buttons[PS3_LEFT_STICK] && mPreviousJoy.buttons[PS3_RIGHT_STICK]) // shift down one mode
	{
		if (getCurrentMode())
			getCurrentMode()->onDeactivate();
		mPS3ModeIndex = (mPS3ModeIndex == 0 ? mPS3ModeCount : mPS3ModeIndex) - 1;
		if (getCurrentMode())
			getCurrentMode()->onActivate();
	}

	if (getCurrentMode() == NULL)
	{
		ROS_ERROR("PS3Mode with index %d does not exist!", mPS3ModeIndex);
		mPreviousJoy = joy;
		return;
	}

	getCurrentMode()->handleController(mPreviousJoy, joy);
	mPreviousJoy = joy;
}

void PS3Controller::addMode(ControllerMode *mode)
{
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
	ps3Controller.spin();
}
