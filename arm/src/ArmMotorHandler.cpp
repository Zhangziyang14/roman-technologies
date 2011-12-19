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

/** threemxl run callback
 *
 * Keeps the threemxl awake so it does not interrupt setPos() commands
 */
void ArmMotorHandler::Run()
{
	while(ros::ok())
	{
		mShoulderMotor.getStatus();
		usleep(500000);
	}
}

/** Constructor
 *
 * @param	char*	path	path that will be passed to Motor::init()
 */
ArmMotorHandler::ArmMotorHandler(char *path) : mNodeHandle("~"), mShoulderMotor(MID_ARM_SHOULDER), mSideMotor(MID_ARM_SIDEWAYS)
{
	// Initialise publisher(s)
	mShoulderAngleSub = mNodeHandle.subscribe("/ShoulderTopic", 10, &ArmMotorHandler::shoulderCB, this);

	// Initialize motors & set position control mode
	mShoulderMotor.init(path);
	mSideMotor.init(path);

	mShoulderMotor.setMode(CM_POSITION_MODE);
	mSideMotor.setMode(CM_POSITION_MODE);

	// thread to fix 3mxel setPos bug
	boost::thread thread(&ArmMotorHandler::Run, this);

	// initialize motor to zero position
	init();

	ROS_INFO("ArmMotorHandler successfully initialized");
}

void ArmMotorHandler::init()
{
	//FIN
}

/** Callbacks **/

/** ShoulderTopic callback
 *
 * @param 	const std_msgs::Float64		msg		ros message containing the wished shoulder position
 *
 * @return	void
 */
void ArmMotorHandler::shoulderCB(const std_msgs::Float64& msg)
{
	ROS_INFO("Setting shoulder to position [%.4f]", msg.data);
	double motorPos = msg.data * SHOULDERMOTOR_CORRECTION_FACTOR;
	motorPos = motorPos * SHOULDERMOTOR_TRANSMISSION_RATIO;
	ROS_INFO("\t motor position is [%.4f]", motorPos);
	mShoulderMotor.setAngle(motorPos, 0.5, 0.5);
}
