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
		mShoulderMotor.update();
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

	// initialize motor to zero position
	/*
	if(init())
		ROS_INFO("ArmMotorHandler successfully initialized");
	else
	{
		ROS_INFO("Error initializing ArmMotorHandler: failed to determine arm position");
		return;
	}
	*/
	ROS_INFO("Error initializing ArmMotorHandler: 3mxel prevents proper turning direction");

	// thread to fix 3mxel setPos bug
	boost::thread thread(&ArmMotorHandler::Run, this);
}


bool ArmMotorHandler::init()
{
	ROS_INFO("Initializing arm to starting position...");

	mShoulderMotor.setMode(CM_EXT_INIT_MODE);
	mShoulderMotor.setTorque(EXT_INIT_MODE_TORQUE);					 //TODO set negative!

	while(mShoulderMotor.getStatus()== M3XL_STATUS_INITIALIZE_BUSY); // wait untill initializing is done

	return (mShoulderMotor.getStatus() == M3XL_STATUS_INIT_DONE);
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
	double targetPos = msg.data;

	ROS_INFO("Setting shoulder to position [%.4f]...", targetPos);

	if(targetPos > SHOULDERMOTOR_MAX_ANGLE)
	{
		ROS_WARN("Desired position exceeds upper angle limit [%.4f]", SHOULDERMOTOR_MAX_ANGLE);
		targetPos = SHOULDERMOTOR_MAX_ANGLE;
	}
	else if(targetPos < SHOULDERMOTOR_MIN_ANGLE)
	{
		ROS_WARN("Desired position exceeds lower angle limit [%.4f]", SHOULDERMOTOR_MIN_ANGLE);
		targetPos = SHOULDERMOTOR_MIN_ANGLE;
	}


	double motorPos = targetPos * SHOULDERMOTOR_CORRECTION_FACTOR;
	motorPos = motorPos * SHOULDERMOTOR_TRANSMISSION_RATIO;
	ROS_INFO("\t motor position is [%.4f]", motorPos);
	mShoulderMotor.setAngle(motorPos, 0.5, 0.5);
}
