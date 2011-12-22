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
	// Initialise subscribers
	mShoulderAngleSub	= mNodeHandle.subscribe("/ShoulderTopic", 10, &ArmMotorHandler::shoulderCB, this);
	mSideJointAngleSub 	= mNodeHandle.subscribe("/SideJointTopic", 10, &ArmMotorHandler::sideJointCB, this);
	mArmJointPosSub		= mNodeHanlde.subscribe("/ArmPositionTopic", 10, &ArmMotorHandler::armPosCB, this);

	// Initialize motors
	mShoulderMotor.init(path);
	mShoulderMotor.setEncoderCount(SHOULDERMOTOR_ENCODER_COUNT * SHOULDERMOTOR_CORRECTION_FACTOR);

	mSideMotor.init(path);
	mSideMotor.setEncoderCount(SIDEMOTOR_ENCODER_COUNT * SIDEMOTOR_CORRECTION_FACTOR);


	// initialize arm to zero position (joints will move to their zero-switches)
	if(init())
		ROS_INFO("ArmMotorHandler successfully initialized");
	else
	{
		ROS_INFO("Error initializing ArmMotorHandler: failed to determine arm position");
		return;
	}

	mShoulderMotor.setMode(CM_POSITION_MODE);
	mSideMotor.setMode(CM_POSITION_MODE);

	// thread to fix 3mxel setPos bug
	boost::thread thread(&ArmMotorHandler::Run, this);
}


bool ArmMotorHandler::init()
{
	ROS_INFO("Initializing arm to starting position...");

	mShoulderMotor.setMode(CM_EXT_INIT_MODE);					//Set to external_init mode
	mShoulderMotor.setAcceleration(EXT_INIT_MODE_ACCEL);		//Set acceleration for initialization
	mShoulderMotor.setSpeed(EXT_INIT_MODE_SHOULDER_SPEED);		//Set speed for initialization
	mShoulderMotor.setTorque(EXT_INIT_MODE_TORQUE);				//Set torque to begin initialization

	mSideMotor.setMode(CM_EXT_INIT_MODE);						//Set to external_init mode
	mSideMotor.setAcceleration(EXT_INIT_MODE_ACCEL);			//Set acceleration for initialization
	mSideMotor.setSpeed(EXT_INIT_MODE_SIDEJOINT_SPEED);			//Set speed for initialization
	mSideMotor.setTorque(EXT_INIT_MODE_TORQUE);					//Set torque to begin initialization


	// wait untill initializing is done
	while((mShoulderMotor.getStatus() == M3XL_STATUS_INITIALIZE_BUSY) || (mSideMotor.getStatus() == M3XL_STATUS_INITIALIZE_BUSY));

	return ((mShoulderMotor.getStatus() == M3XL_STATUS_INIT_DONE) && (mSideMotor.getStatus() == M3XL_STATUS_INIT_DONE));
}



void ArmMotorHandler::setShoulderAngle(double angle)
{
	ROS_INFO("Setting shoulder to position [%.4f]...", angle);

	if(angle > SHOULDERMOTOR_MAX_ANGLE)
	{
		ROS_WARN("Desired shoulder position exceeds upper angle limit [%.4f]", SHOULDERMOTOR_MAX_ANGLE);
		angle = SHOULDERMOTOR_MAX_ANGLE;
	}
	else if(angle < SHOULDERMOTOR_MIN_ANGLE)
	{
		ROS_WARN("Desired shoulder position exceeds lower angle limit [%.4f]", SHOULDERMOTOR_MIN_ANGLE);
		angle = SHOULDERMOTOR_MIN_ANGLE;
	}
	mShoulderMotor.setAngle(angle, DEFAULT_SPEED, DEFAULT_ACCEL);
}

void ArmMotorHandler::setSideJointAngle(double angle)
{
	ROS_INFO("Setting sideJoint to position [%.4f]...", angle);

	if(angle > SIDEJOINT_MAX_ANGLE)
	{
		ROS_WARN("Desired sideJoint position exceeds upper angle limit [%.4f]", SIDEJOINT_MAX_ANGLE);
		angle = SIDEJOINT_MAX_ANGLE;
	}
	else if(angle < SIDEJOINT_MIN_ANGLE)
	{
		ROS_WARN("Desired sideJoint position exceeds lower angle limit [%.4f]", SIDEJOINT_MIN_ANGLE);
		angle = SIDEJOINT_MIN_ANGLE;
	}
	mSideMotor.setAngle(angle, DEFAULT_SPEED, DEFAULT_ACCEL);

}


/* * * * * Callbacks * * * * */


/** ShoulderTopic callback
 *
 * @param 	const std_msgs::Float64		msg		ros message containing the wished shoulder position
 *
 * @return	void
 */
void ArmMotorHandler::shoulderCB(const std_msgs::Float64& msg)
{
	setShoulderAngle(msg.data);
}


/** SideJointTopic callback
 *
 * @param 	const std_msgs::Float64		msg		ros message containing the wished sidejoint position
 *
 * @return	void
 */
void ArmMotorHandler::sideJointCB(const std_msgs::Float64& msg)
{
	setSideJointAngle(msg.data);
}
