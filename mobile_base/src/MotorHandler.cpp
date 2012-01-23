#include <mobile_base/MotorHandler.h>

/**
 * Publishes the linear and angular speed of the robot
 */
void MotorHandler::publishRobotSpeed()
{
	mRightMotorSpeed = mRightMotor.getRotationSpeed();
	mLeftMotorSpeed = mLeftMotor.getRotationSpeed();

	// linear speed is the average of both motor speeds and it is converted from rad/s to m/s
	mCurrentSpeed.linear.x  = (mRightMotorSpeed + mLeftMotorSpeed) * WHEEL_RADIUS / 2.0;
	mCurrentSpeed.angular.z = (mRightMotorSpeed - mLeftMotorSpeed) * WHEEL_RADIUS / 0.25 /2.0; //* getBaseRadius();

	mSpeedPub.publish(mCurrentSpeed);
}

/**
 * Controls the motors based on the received position.
 */
void MotorHandler::positionCB(const mobile_base::position& msg)
{
	mRightMotor.brake();
	mLeftMotor.brake();

	double currentRightPosition = mRightMotor.getPosition();
	double currentLeftPosition = mLeftMotor.getPosition();

	mLock = true;
	mRightMotor.setMode(CM_POSITION_MODE);
	mLeftMotor.setMode(CM_POSITION_MODE);

	//ROS_INFO("left: %f, right: %f, front_l: %d, front_r: %d, rear_l: %d, rear_r: %d", msg.left, msg.right, mFrontLeftCenter, mFrontRightCenter, mRearLeft, mRearRight);
	if((msg.left > 0 && msg.right > 0  && mFrontLeftCenter > msg.left*100 && mFrontRightCenter > msg.left*100) || 
			(msg.left < 0 && msg.right < 0 && mRearLeft > std::abs(msg.left*100) && mRearRight > std::abs(msg.left*100)) ||
			msg.left != msg.right)
	{
		mRightMotor.setPosition(currentRightPosition + msg.right);
		mLeftMotor.setPosition(currentLeftPosition + msg.left);
	}
}

/**
 * Controls the speed of the motors based on the received twist message.
 */
void MotorHandler::moveCB(const geometry_msgs::Twist& msg)
{
	//Unblock the robot
	if(mCurrentSpeed.linear.x == 0)
		mLock = false;

	double converted_right	= ZERO_SPEED, converted_left = ZERO_SPEED;

	//Convert linear speed to motor speeds in [rad/s]
	double vel_linear = msg.linear.x / WHEEL_RADIUS;
	double vel_angular = msg.angular.z * (BASE_RADIUS / WHEEL_RADIUS) * 2;

	converted_left  = vel_linear - 0.5*vel_angular;
	converted_right = vel_linear + 0.5*vel_angular;
/*
	if(mLock)
	{
		//ROS_INFO("linear.x: %f, front_l: %d, front_r: %d, rear_l: %d, rear_r: %d", mCurrentSpeed.linear.x, mFrontLeftCenter, mFrontRightCenter, mRearLeft, mRearRight);
		//Disable motors when boxed or when in position mode
		if((mCurrentSpeed.linear.x > ZERO_SPEED && (mFrontLeftCenter < 80 || mFrontRightCenter < 80)) || (mCurrentSpeed.linear.x < ZERO_SPEED && (mRearLeft < 50 || mRearRight < 50)))
		{
			mRightMotor.brake();
			mLeftMotor.brake();
		}
	}
	else
	{

	}
	*/

	mRightMotor.setSpeed(converted_right);
	mLeftMotor.setSpeed(converted_left);
}

/**
 * Called when a Twist message is received over the motor topic.
 */
void MotorHandler::tweakCB(const mobile_base::tweak& msg)
{
	Motor *motor = mMotorId == MID_LEFT ? &mLeftMotor : &mRightMotor;

	// scale the current P-I-D value
	if (msg.scaleUp || msg.scaleDown)
	{
		motor->mPID[mPIDFocus] += msg.scaleDown ? -PID_TWEAK_STEP : PID_TWEAK_STEP;
		motor->mPID[mPIDFocus] = std::max(0.0, motor->mPID[mPIDFocus]);
		motor->updatePID();
		motor->printPID();
	}

	// toggle the P-I-D focus
	if (msg.toggleForward || msg.toggleBackward)
	{
		// toggle through P-I-D
		mPIDFocus = PIDParameter(mPIDFocus + (msg.toggleBackward ? -1 : 1));
		mPIDFocus = PIDParameter(mPIDFocus == -1 ? PID_PARAM_MAX - 1 : mPIDFocus % PID_PARAM_MAX);

		switch (mPIDFocus)
		{
		case PID_PARAM_P: ROS_INFO("P"); break;
		case PID_PARAM_I: ROS_INFO("I"); break;
		case PID_PARAM_D: ROS_INFO("D"); break;
		default: break;
		}
	}

	// change selected motor?
	if (msg.motorID)
	{
		mMotorId = MotorId(msg.motorID);
		ROS_INFO("SWITCHED TO %s MOTOR", mMotorId == MID_LEFT ? "LEFT" : "RIGHT");
	}
}

/**
 * Negate the ultrasone sensors in the ultrasone array
 */
void negateUltrasone(int length, int* ultrasone[])
{
	for(int i = 0; i < length ; i++)
		*ultrasone[i] = 150;
}

/**
 *	Reads distances from ultrasone sensors
 */
//TODO: Turn off the unused ultrasone sensors in arduino code
void MotorHandler::ultrasoneCB(const mobile_base::sensorFeedback& msg)
{
	if(mCurrentSpeed.linear.x > 0)
	{
		int* ultrasone[2] = {&mRearLeft, &mRearRight};
		negateUltrasone(2, ultrasone);

		//TODO: mFrontLeftCenter and frontCenterLeft are flipped in arduino code.. Fix this! (vice versa for mFrontRightCenter and frontCenterRight)

		mFrontLeftCenter	= msg.frontCenterLeft;
		mFrontRightCenter	= msg.frontCenterRight;

		mFrontCenterRight	= msg.frontRightCenter;
		mFrontCenterLeft	= msg.frontLeftCenter;

		mFrontLeft 			= msg.frontLeft;
		mFrontRight			= msg.frontRight;
	}
	else if(mCurrentSpeed.linear.x < 0)
	{
		int* ultrasone[6] = {&mFrontLeftCenter, &mFrontRightCenter, &mFrontCenterRight, &mFrontCenterLeft, &mFrontLeft, &mFrontRight};
		negateUltrasone(6, ultrasone);

		mRearLeft			= msg.rearLeft;
		mRearRight			= msg.rearRight;
	}
	else
	{
		mFrontLeftCenter	= msg.frontCenterLeft;
		mFrontRightCenter	= msg.frontCenterRight;

		mFrontCenterRight	= msg.frontRightCenter;
		mFrontCenterLeft	= msg.frontLeftCenter;

		mFrontLeft 			= msg.frontLeft;
		mFrontRight			= msg.frontRight;

		mRearLeft			= msg.rearLeft;
		mRearRight			= msg.rearRight;
	}

	mLeft				= msg.left;
	mRight				= msg.right;

	//ROS_INFO("left %d, frontLeft %d, frontCenterLeft %d, frontLeftCenter %d, frontRightCenter %d, frontCenterRight %d, right %d", mLeft, mFrontLeft, mFrontCenterLeft, mFrontLeftCenter, mFrontRightCenter, mFrontCenterRight, mRight);
	//ROS_INFO("rearLeft %d, rearRight %d", mRearLeft, mRearRight);
}

/**
 * Initialise MotorHandler and its attributes.
 */
void MotorHandler::init(char *path)
{
	//Initialise publishers
	mSpeedPub = mNodeHandle.advertise<geometry_msgs::Twist>("/speedFeedbackTopic", 1);

	//Initialise subscribers
	mTweakPIDSub 	= mNodeHandle.subscribe("/tweakTopic", 10, &MotorHandler::tweakCB, this);
	mTwistSub 		= mNodeHandle.subscribe("/cmd_vel", 10, &MotorHandler::moveCB, this);
	mPositionSub 	= mNodeHandle.subscribe("/positionTopic", 10, &MotorHandler::positionCB, this);
	mUltrasoneSub   = mNodeHandle.subscribe("/sensorFeedbackTopic", 10, &MotorHandler::ultrasoneCB, this);

	mLock = false;

	mLeftMotor.init(path);
	mRightMotor.init(path);

	//Initialise distances from ultrasone sensors
	mFrontLeftCenter 	= ULTRASONE_MAX_RANGE;
	mFrontRightCenter 	= ULTRASONE_MAX_RANGE;
	mFrontCenterLeft 	= ULTRASONE_MAX_RANGE;
	mFrontCenterRight 	= ULTRASONE_MAX_RANGE;
	mRearLeft 			= ULTRASONE_MAX_RANGE;
	mFrontLeft			= ULTRASONE_MAX_RANGE;
	mRearRight 			= ULTRASONE_MAX_RANGE;
	mFrontRight 		= ULTRASONE_MAX_RANGE;
	mRight 				= ULTRASONE_MAX_RANGE;
	mLeft 				= ULTRASONE_MAX_RANGE;

	ROS_INFO("Initialising completed.");
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "BaseMotorHandler");

	char *path=NULL;
	if (argc == 2)
		path = argv[1];

	MotorHandler motorHandler;
	motorHandler.init(path);

	ros::Rate looprate(50);

	while(ros::ok())
	{
		motorHandler.publishRobotSpeed();
		ros::spinOnce();
		looprate.sleep();
	}

	return 0;
}
