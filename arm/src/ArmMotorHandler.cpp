#include <ArmMotorHandler.h>



int main(int argc, char **argv)
{
	ros::init(argc, argv, "ArmMotorHandler");

	char *path=NULL;
	if (argc == 2)
		path = argv[1];

	ArmMotorHandler armHandler(path);

	while(ros::ok())
	{
		//armHandler.doSomethingFancy();
		ros::spinOnce();
	}

	return 0;
}


ArmMotorHandler::ArmMotorHandler(char *path) : mNodeHandle("~"), mShoulderMotor(MID_ARM_SHOULDER), mSideMotor(MID_ARM_SIDEWAYS)
{
	//FIN Initialise publishers
	mShoulderAngleSub = mNodeHandle.subscribe("/ShoulderTopic", 10, &ArmMotorHandler::shoulderCB, this);

	mShoulderMotor.init(path);
	mSideMotor.init(path);


	ROS_INFO("ArmMotorHandler successfully initialized");
}


/** Callbacks **/
void ArmMotorHandler::shoulderCB(const std_msgs::Float64& msg)
{
	mShoulderMotor.setMode(CM_POSITION_MODE);

	double motorPos = msg.data * 3.0;		//CLEAN
	mShoulderMotor.setPosition(motorPos);
}
