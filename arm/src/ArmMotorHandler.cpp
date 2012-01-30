#include <ArmMotorHandler.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "ArmMotorHandler");

	char *path=NULL;
	if (argc == 2)
		path = argv[1];

	ArmMotorHandler armHandler(path);

	int sleep_rate;
	armHandler.getNodeHandle()->param<int>("node_sleep_rate", sleep_rate, 50);
	ros::Rate sleep(sleep_rate);

	while(ros::ok())
	{
		armHandler.publishArmPosition();
		armHandler.publishArmSpeed();
		ros::spinOnce();
		sleep.sleep();
	}

	return 0;
}

/**
 * Stops all motors and locks them until unlock command is received
 */
void ArmMotorHandler::stopCB(const std_msgs::Bool &msg)
{
	if(msg.data)
	{
		mShoulderMotor.setMode(CM_STOP_MODE);
		mSideMotor.setMode(CM_STOP_MODE);
	}

	mShoulderMotor.lock(msg.data);
	mSideMotor.lock(msg.data);
}

/**
 * Publishes the current positions of both motors
 */
void ArmMotorHandler::publishArmPosition()
{
	mCurrentShoulderJointPos = getShoulderAngle();
	mCurrentSideJointPos 	 = getSideJointAngle();

	arm::armJointPos joint_msg;

	joint_msg.upper_joint 	= mCurrentShoulderJointPos;
	joint_msg.wrist_joint 	= mCurrentSideJointPos;

	mArmPosFeedbackPub.publish(joint_msg);
}

/**
 * Publishes the current speeds of both motors
 */
void ArmMotorHandler::publishArmSpeed()
{
	mCurrentShoulderJointSpeed 	 = getShoulderSpeed();
	mCurrentSideJointSpeed	 	 = getSideJointSpeed();

	arm::armJointPos speed_msg;

	speed_msg.upper_joint 	= mCurrentShoulderJointSpeed;
	speed_msg.wrist_joint 	= mCurrentSideJointSpeed;

	mArmSpeedFeedbackPub.publish(speed_msg);
}

/** threemxl run callback
 *
 * Keeps the threemxl awake so it does not interrupt setPos() commands
 * Not needed anymore since publishRobotSpeed always talks to threemxl, so watchdog won't kick in anymore

void ArmMotorHandler::Run()
{
	while(ros::ok())
	{
		mShoulderMotor.update();
		mSideMotor.update();
		usleep(500000);
	}
}
 */

/** Constructor
 *
 * @param	char*	path	path that will be passed to Motor::init()
 */
ArmMotorHandler::ArmMotorHandler(char *path) : mNodeHandle("~"), mShoulderMotor(MID_ARM_SHOULDER), mSideMotor(MID_ARM_SIDEWAYS)
{
	// Initialize motors
	mShoulderMotor.init(path);
	mShoulderMotor.setEncoderCount(SHOULDERMOTOR_ENCODER_COUNT * SHOULDERMOTOR_CORRECTION_FACTOR);

	mSideMotor.init(path);
	mSideMotor.setEncoderCount(SIDEMOTOR_ENCODER_COUNT * SIDEMOTOR_CORRECTION_FACTOR);


	// initialize arm to zero position (joints will move to their zero-switches)
	if(init())
	{
		ROS_INFO("ArmMotorHandler successfully initialised");

		//Initialise publishers
		mArmPosFeedbackPub		= mNodeHandle.advertise<arm::armJointPos>("/armJointPositionFeedbackTopic", 1);
		mArmSpeedFeedbackPub	= mNodeHandle.advertise<arm::armJointPos>("/armJointSpeedFeedbackTopic", 1);

		// Initialise subscribers
		mShoulderAngleSub	= mNodeHandle.subscribe("/ShoulderTopic", 10, &ArmMotorHandler::shoulderCB, this);
		mSideJointAngleSub 	= mNodeHandle.subscribe("/SideJointTopic", 10, &ArmMotorHandler::sideJointCB, this);
		mArmJointPosSub		= mNodeHandle.subscribe("/armJointPositionTopic", 10, &ArmMotorHandler::armPosCB, this);
		mStopSubscriber		= mNodeHandle.subscribe("/emergencyStop", 1, &ArmMotorHandler::stopCB, this);

		mShoulderMotor.setMode(CM_POSITION_MODE);
		mSideMotor.setMode(CM_POSITION_MODE);
		usleep(500000);

		setShoulderAngle(SHOULDERMOTOR_START_POS);
		while((getShoulderAngle()) < (SHOULDERMOTOR_MIN_ANGLE - SAFETY_OFFSET));

		setSideJointAngle(SIDEJOINT_START_POS);

		mCurrentShoulderJointPos = getShoulderAngle();
		mCurrentSideJointPos = getSideJointAngle();
	}
	else
		ROS_WARN("Error initializing ArmMotorHandler: failed to determine arm position. Please make sure the arm is not at its limits and try again.");
}


bool ArmMotorHandler::init()
{
	ROS_INFO("Initializing arm to starting position...");

	mSideMotor.setMode(CM_EXT_INIT_MODE);														//Set to external_init mode
	mSideMotor.setAcceleration(EXT_INIT_MODE_ACCEL / SIDEMOTOR_CORRECTION_FACTOR);				//Set acceleration for initialization
	mSideMotor.setSpeed(EXT_INIT_MODE_SIDEJOINT_SPEED / SIDEMOTOR_CORRECTION_FACTOR);			//Set speed for initialization
	mSideMotor.setTorque(EXT_INIT_MODE_TORQUE);													//Set torque to begin initialization

	// wait untill side arm initialization is done
	while(mSideMotor.getStatus() == M3XL_STATUS_INITIALIZE_BUSY);
	if(mSideMotor.getStatus() != M3XL_STATUS_INIT_DONE)
	{
		ROS_ERROR("Error: %s", C3mxl::translateErrorCode(mSideMotor.getStatus()));
		return false;
	}

	mShoulderMotor.setMode(CM_EXT_INIT_MODE);													//Set to external_init mode
	mShoulderMotor.setAcceleration(EXT_INIT_MODE_ACCEL / SHOULDERMOTOR_CORRECTION_FACTOR);		//Set acceleration for initialization
	mShoulderMotor.setSpeed(EXT_INIT_MODE_SHOULDER_SPEED / SHOULDERMOTOR_CORRECTION_FACTOR);	//Set speed for initialization
	mShoulderMotor.setTorque(EXT_INIT_MODE_TORQUE);												//Set torque to begin initialization

	// wait untill shoulder initialation is done
	while(mShoulderMotor.getStatus() == M3XL_STATUS_INITIALIZE_BUSY);
	if(mShoulderMotor.getStatus() != M3XL_STATUS_INIT_DONE)
		ROS_ERROR("Error: %s", C3mxl::translateErrorCode(mShoulderMotor.getStatus()));
	return (mShoulderMotor.getStatus() == M3XL_STATUS_INIT_DONE);
}

void ArmMotorHandler::setShoulderAngle(double angle)
{
	if (isnan(angle))
	{
		ROS_WARN("Received nan shoulder angle.");
		return;
	}

	ROS_INFO("Setting shoulder to position [%.3f]...", angle);

	if(angle > SHOULDERMOTOR_MAX_ANGLE)
	{
		ROS_WARN("Desired shoulder position [%.3f] exceeds upper angle limit [%.3f]", angle, SHOULDERMOTOR_MAX_ANGLE);
		angle = SHOULDERMOTOR_MAX_ANGLE;
	}
	else if(angle < SHOULDERMOTOR_MIN_ANGLE)
	{
		ROS_WARN("Desired shoulder position [%.3f] exceeds lower angle limit [%.3f]", angle, SHOULDERMOTOR_MIN_ANGLE);
		angle = SHOULDERMOTOR_MIN_ANGLE;
	}
	angle+= SHOULDERMOTOR_OFFSET;		// offset to transform the absolute position to the position relative to the limit switch
	mShoulderMotor.setAngle((angle / SHOULDERMOTOR_CORRECTION_FACTOR), (DEFAULT_SPEED / SHOULDERMOTOR_CORRECTION_FACTOR), (DEFAULT_ACCEL / SHOULDERMOTOR_CORRECTION_FACTOR));
}

void ArmMotorHandler::setSideJointAngle(double angle)
{
	if (isnan(angle))
	{
		ROS_WARN("Received nan side joint angle.");
		return;
	}

	ROS_INFO("Setting sideJoint to position [%.3f]...", angle);

	if(angle > SIDEJOINT_MAX_ANGLE)
	{
		ROS_WARN("Desired sideJoint position [%.3f] exceeds upper angle limit [%.3f]", angle, SIDEJOINT_MAX_ANGLE);
		angle = SIDEJOINT_MAX_ANGLE;
	}
	else if(angle < SIDEJOINT_MIN_ANGLE)
	{
		ROS_WARN("Desired sideJoint position [%.3f] exceeds lower angle limit [%.3f]", angle, SIDEJOINT_MIN_ANGLE);
		angle = SIDEJOINT_MIN_ANGLE;
	}
	angle+= SIDEJOINT_OFFSET;		// offset to transform the absolute position to the position relative to the limit switch
	mSideMotor.setAngle((angle / SIDEMOTOR_CORRECTION_FACTOR), (DEFAULT_SPEED / SIDEMOTOR_CORRECTION_FACTOR), (DEFAULT_ACCEL / SIDEMOTOR_CORRECTION_FACTOR));

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


/** ArmPositionTopic callback
 *
 * @param	const arm::armJointPos		msg		ros message containing the wished positions of both
 * 												the shoulder and the sideJoint
 * @return	void
 */
void ArmMotorHandler::armPosCB(const arm::armJointPos& msg)
{
	setShoulderAngle(msg.upper_joint);
	setSideJointAngle(msg.wrist_joint);
}
