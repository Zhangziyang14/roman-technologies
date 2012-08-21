/*
 * ArmMode.cpp
 *
 *  Created on: Aug 21, 2012
 *      Author: hans
 */

#include "nero_tools/HeadMode.h"

HeadMode::HeadMode(ros::NodeHandle *nodeHandle) :
	ControllerMode(nodeHandle)
{
	mHeadSpeedPub = nodeHandle->advertise<nero_msgs::PitchYaw>("/head/cmd_vel", 1);

	ROS_INFO("[HeadMode] Initialized.");
}

void HeadMode::handleController(std::vector<int> previousButtons, std::vector<float> previousAxes, const sensor_msgs::Joy &joy)
{
	ControllerMode::handleController(previousButtons, previousAxes, joy);

	// speed of the arm
	if (joy.axes[PS3_AXIS_LEFT_HORIZONTAL] || joy.axes[PS3_AXIS_RIGHT_VERTICAL]) // we are sending speed
		sendHeadSpeed(joy.axes[PS3_AXIS_RIGHT_VERTICAL], joy.axes[PS3_AXIS_LEFT_HORIZONTAL]);
	else if (previousAxes[PS3_AXIS_LEFT_HORIZONTAL] || previousAxes[PS3_AXIS_RIGHT_VERTICAL]) // we stopped sending speed
		sendHeadSpeed(0.f, 0.f);
}

void HeadMode::sendHeadSpeed(float pitchScale, float yawScale)
{
	nero_msgs::PitchYaw msg;
	msg.pitch = pitchScale * MAX_PITCH_SPEED;
	msg.yaw = yawScale * MAX_PITCH_SPEED;
	mHeadSpeedPub.publish(msg);
}
