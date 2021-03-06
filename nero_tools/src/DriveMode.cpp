/*
 * DriveMode.cpp
 *
 *  Created on: Aug 17, 2012
 *      Author: hans
 */

#include "nero_tools/DriveMode.h"

DriveMode::DriveMode(ros::NodeHandle *nodeHandle) :
	ControllerMode(nodeHandle), mPingState(true), mTimer(mIOService, boost::posix_time::seconds(1)),
	mLastLinearSpeed(0.f), mLastAngularSpeed(0.f), mActive(false)
{
	mSpeedPub 			= nodeHandle->advertise<geometry_msgs::Twist>("/cmd_vel", 1);
	mArmPosPub			= nodeHandle->advertise<geometry_msgs::Pose>("/cmd_arm_position", 1);
	mGripperStatePub	= nodeHandle->advertise<std_msgs::Bool>("/cmd_gripper_state", 1);
	mPingStatePub		= nodeHandle->advertise<std_msgs::Bool>("/cmd_gripper", 1);

	ROS_INFO("[DriveMode] Initialized.");
}

void DriveMode::handleController(std::vector<int> previousButtons, std::vector<float> previousAxes, const sensor_msgs::Joy &joy)
{
	ControllerMode::handleController(previousButtons, previousAxes, joy);

	// speed of the base
	if (joy.axes[PS3_AXIS_LEFT_HORIZONTAL] || joy.axes[PS3_AXIS_RIGHT_VERTICAL]) // we are sending speed
		sendSpeed(joy.axes[PS3_AXIS_RIGHT_VERTICAL], joy.axes[PS3_AXIS_LEFT_HORIZONTAL]);
	else if (previousAxes[PS3_AXIS_LEFT_HORIZONTAL] || previousAxes[PS3_AXIS_RIGHT_VERTICAL]) // we stopped sending speed
		sendSpeed(0.f, 0.f);

	// arm
	if (pressed(previousButtons, joy, PS3_R1))
		sendArmPosition(ARM_POS_UP_X, ARM_POS_UP_Z);
	if (pressed(previousButtons, joy, PS3_R2))
		sendArmPosition(ARM_POS_DOWN_X, ARM_POS_DOWN_Z);

	// gripper
	if (pressed(previousButtons, joy, PS3_L1))
		sendGripperState(true);
	if (pressed(previousButtons, joy, PS3_L2))
		sendGripperState(false);

	// ping sensor
	if (pressed(previousButtons, joy, PS3_SELECT))
	{
		mPingState = !mPingState;
		sendPingState(mPingState);
	}

}

void DriveMode::sendSpeed(float linearScale, float angularScale)
{
	geometry_msgs::Twist msg;
	msg.linear.x = linearScale * MAX_LINEAR_SPEED;
	msg.angular.z = angularScale * MAX_ANGULAR_SPEED;
	mSpeedPub.publish(msg);

	mLastLinearSpeed = msg.linear.x;
	mLastAngularSpeed = msg.angular.z;

	//ROS_INFO("[DriveMode] Sending speed (%f, %f)", msg.linear.x, msg.angular.z);
}

void DriveMode::sendArmPosition(float x, float z)
{
	geometry_msgs::Pose msg;
	msg.position.x = x;
	msg.position.z = z;
	mArmPosPub.publish(msg);

	ROS_INFO("[DriveMode] Sending arm position (%f, %f)", x, z);
}

void DriveMode::sendGripperState(bool state)
{
	std_msgs::Bool msg;
	msg.data = state;
	mGripperStatePub.publish(msg);

	ROS_INFO("[DriveMode] Sending gripper state (%s)", state == true ? "true" : "false");
}

void DriveMode::sendPingState(bool state)
{
	std_msgs::Bool msg;
	msg.data = state;
	mPingStatePub.publish(msg);

	ROS_INFO("[DriveMode] Sending ping state (%s)", state == true ? "true" : "false");
}

void DriveMode::sendSpeedEvent()
{
	ROS_INFO("Speed Event.");
	if (mActive)
	{
		mTimer.expires_at(mTimer.expires_at() + boost::posix_time::seconds(1));
		mTimer.async_wait(boost::bind(&DriveMode::sendSpeedEvent, this));

		sendSpeed(mLastLinearSpeed, mLastAngularSpeed);
	}
}

void DriveMode::onActivate()
{
	ROS_INFO("[DriveMode] Activated.");

	//boost::asio::deadline_timer t(mIOService, boost::posix_time::seconds(1));
	//mTimer.async_wait(boost::bind(&DriveMode::sendSpeedEvent, this));

	mActive = true;
}

void DriveMode::runOnce()
{
	//mIOService.run_one();
}

void DriveMode::onDeactivate()
{
	ROS_INFO("[DriveMode] Deactivated.");

	sendSpeed(0.f, 0.f);
	mActive = false;
}
