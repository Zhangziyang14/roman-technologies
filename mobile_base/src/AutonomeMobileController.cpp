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
		mPathFollower.updatePath();

		refreshRate.sleep();
		ros::spinOnce();
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "AutonomeMobileController");

	AutonomeMobileController autonomeMobileController;

	autonomeMobileController.spin();
	return 0;
}
