/*
 * AutonomeMobileController.cpp
 *
 *  Created on: 2012-01-20
 *      Author: wilson
 */
#include "mobile_base/AutonomeMobileController.h"

/**
 * Main follow loop.
 */
void AutonomeMobileController::spin()
{
	ros::Rate refreshRate(mRefreshRate);

	while (ros::ok())
	{
		mPathHandler.updatePath();

		refreshRate.sleep();
		ros::spinOnce();
	}
}

void AutonomeMobileController::init()
{
	//Initialise Publishers
	mPositionPub = mNodeHandle.advertise<nero_msgs::MotorPosition>("/positionTopic", 1);

	//Initialise Subscribers
	mPositionSub = mNodeHandle.subscribe("/cmd_mobile_position", 1, &AutonomeMobileController::positionCB, this);
	mTurnSub	 = mNodeHandle.subscribe("/cmd_mobile_turn", 1, &AutonomeMobileController::turnCB, this);

	ROS_INFO("Initialising complete");
}

/**
 * Makes Eva move forward or backwards
 */
void AutonomeMobileController::positionCB(const std_msgs::Float32& msg)
{
	nero_msgs::MotorPosition pos_msg;

	pos_msg.left = msg.data;
	pos_msg.right = msg.data;

	mPositionPub.publish(pos_msg);
}

/**
 * Makes Eva turn around her axis
 */
void AutonomeMobileController::turnCB(const std_msgs::Float32& msg)
{
	nero_msgs::MotorPosition pos_msg;

	// turning with the clock = negative
	pos_msg.left = -1 * msg.data * BASE_RADIUS;
	pos_msg.right = msg.data * BASE_RADIUS;

	mPositionPub.publish(pos_msg);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "AutonomeMobileController");

	AutonomeMobileController autonomeMobileController;

	autonomeMobileController.init();
	autonomeMobileController.spin();

	return 0;
}
