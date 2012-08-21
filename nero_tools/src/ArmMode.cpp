/*
 * ArmMode.cpp
 *
 *  Created on: Aug 21, 2012
 *      Author: parallels
 */

#include "nero_tools/ArmMode.h"

ArmMode::ArmMode(ros::NodeHandle *nodeHandle) :
	ControllerMode(nodeHandle)
{
	mArmSpeedPub = nodeHandle->advertise<nero_msgs::ArmJoint>("/arm/cmd_vel", 1);

	ROS_INFO("[ArmMode] Initialized.");
}

void ArmMode::handleController(std::vector<int> previousButtons, std::vector<float> previousAxes, const sensor_msgs::Joy &joy)
{
	ControllerMode::handleController(previousButtons, previousAxes, joy);

	// speed of the arm
	if (joy.axes[PS3_AXIS_LEFT_HORIZONTAL] || joy.axes[PS3_AXIS_RIGHT_VERTICAL]) // we are sending speed
		sendArmSpeed(joy.axes[PS3_AXIS_LEFT_HORIZONTAL], joy.axes[PS3_AXIS_RIGHT_VERTICAL]);
	else if (previousAxes[PS3_AXIS_LEFT_HORIZONTAL] || previousAxes[PS3_AXIS_RIGHT_VERTICAL]) // we stopped sending speed
		sendArmSpeed(0.f, 0.f);
}

void ArmMode::sendArmSpeed(float wristScale, float upperScale)
{
	nero_msgs::ArmJoint msg;
	msg.wrist_joint = wristScale * MAX_WRIST_SPEED;
	msg.upper_joint = upperScale * MAX_UPPER_SPEED;
	mArmSpeedPub.publish(msg);
}
