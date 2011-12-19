#include "AutonomeArmController.h"






void AutonomeArmController::init()
{
	ROS_INFO("AutonomeArmController initialized");
}





int main(int argc, char** argv)
{
	ros::init(argc, argv, "AutonomeArmController");

	AutonomeArmController* ArmController = new AutonomeArmController();
	ArmController->init();

	while(ros::ok())
	{
		ros::spin();
	}

	return EXIT_SUCCESS;	// 0
}
