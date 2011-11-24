#include <MotorHandler.h>

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
void MotorHandler::positionCB(const std_msgs::Float64& msg)
{
	double currentRightPosition = mRightMotor.getPosition();
	double currentLeftPosition = mLeftMotor.getPosition();
	//double newRightPosition, newLeftPosition;

	mLock = true;
	mRightMotor.setMode(CM_POSITION_MODE);
	mLeftMotor.setMode(CM_POSITION_MODE);

	mRightMotor.setPosition(currentRightPosition + msg.data);
	mLeftMotor.setPosition(currentLeftPosition + msg.data);
}

/**
 * Controls the speed of the motors based on the received twist message.
 */
void MotorHandler::moveCB(const mobile_base::BaseMotorControl& msg)
{
	//Checks whether the robot was not already moving
	if(mCurrentSpeed.linear.x == 0)
		mLock = false;

	if(mLock == false)
	{
		double left_vel  = msg.left_motor_speed;
		double right_vel = msg.right_motor_speed;

		if (left_vel == 0.0 && right_vel == 0.0)
		{
			double vel_linear = msg.twist.linear.x / WHEEL_RADIUS;
			double vel_angular = msg.twist.angular.z / (BASE_RADIUS / WHEEL_RADIUS);

			left_vel = vel_linear - vel_angular;
			right_vel = vel_linear + vel_angular;
		}
		/*
		// disable positive speeds ?
		if (mDisableForwardMotion)
		{
			left_vel = std::min(0.0, left_vel);
			right_vel = std::min(0.0, right_vel);
		}
		// disable negative speeds ?
		if (mDisableBackwardMotion)
		{
			left_vel = std::max(0.0, left_vel);
			right_vel = std::max(0.0, right_vel);
		}
		 */
		mRightMotor.setSpeed(right_vel);
		mLeftMotor.setSpeed(left_vel);
	}
}

void MotorHandler::dummyCB(const std_msgs::Float64& msg)
{

	/*
		// disable positive speeds ?
		if (mDisableForwardMotion)
		{
			left_vel = std::min(0.0, left_vel);
			right_vel = std::min(0.0, right_vel);
		}
		// disable negative speeds ?
		if (mDisableBackwardMotion)
		{
			left_vel = std::max(0.0, left_vel);
			right_vel = std::max(0.0, right_vel);
		}
	 */

	while(true)
	{
		double left_speed = msg.data, right_speed = msg.data;

		//if(msg.data > 0)
		//{
		//	if(mFrontRight < 100)
				left_speed  = std::min(msg.data-msg.data*((100.0-mFrontRight)/100.0),msg.data);

		//	if(mFront < 100)
			//	right_speed = std::min(msg.data-msg.data*((100.0-mFront)/100.0),msg.data);

//			if(mFrontLeft < 100)
				right_speed = std::min(msg.data-msg.data*((100.0-mFrontLeft)/100.0),msg.data);
	//	}
/*
		if(msg.data < 0)
		{
			right_speed = std::min(msg.data-msg.data*((100.0-mRearRight)/100.0),msg.data);
			left_speed  = std::min(msg.data-msg.data*((100.0-mRearLeft)/100.0),msg.data);
		}*/

		/*if(mFrontLeft < 5+
		 * 0 && mFrontLeft > 0)
		{
			right_speed = 0;
		}

		if(mFrontRight < 50 && mFrontRight > 0)
		{
			left_speed = 0;
		}*/

		mRightMotor.setSpeed(right_speed);
		mLeftMotor.setSpeed(left_speed);
		ros::spinOnce();
	}
}


/**
 * Called when a Twist message is received over the motor topic.
 */
void MotorHandler::tweakCB(const mobile_base::tweak msg)
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
 * Called when a forward/backward motion needs to be disabled/enabled

void MotorHandler::disableMotorCB(const mobile_base::DisableMotor &msg)
{
	mDisableForwardMotion = msg.disableForward;
	mDisableBackwardMotion = msg.disableBackward;

	// are we driving in a direction now forbidden ? Stop the robot.
	if ((mCurrentSpeed.linear.x > 0 && mDisableForwardMotion) || (mCurrentSpeed.linear.x < 0 && mDisableBackwardMotion))
	{
		mLeftMotor.setSpeed(0.0);
		mRightMotor.setSpeed(0.0);
	}

	if (mDisableForwardMotion)
		ROS_INFO("Disable forward.");
	if (mDisableBackwardMotion)
		ROS_INFO("Disable backward.");
}*/

/**
 *	Reads distances from ultrasone sensors
 */
void MotorHandler::ultrasoneCB(const mobile_base::sensorFeedback& msg)
{
	mFront	 	= msg.frontCenter;
	mFrontLeft 	= msg.frontLeft;
	mFrontRight	= msg.frontRight;

	mRear		= msg.rearCenter;
	mRearLeft	= msg.rearLeft;
	mRearRight	= msg.rearRight;

	mLeft		= msg.left;
	mRight		= msg.right;
}

/**
 * Initialise MotorHandler and its attributes.
 */
void MotorHandler::init(char *path)
{
	//Initialise publishers
	mSpeedPub = mNodeHandle.advertise<geometry_msgs::Twist>("/speedFeedbackTopic", 1);
	mUltrasoneActivatePub = mNodeHandle.advertise<std_msgs::UInt8>("/sensorActivateTopic", 10);

	//Initialise subscribers
	mTweakPIDSub 	= mNodeHandle.subscribe("/tweakTopic", 10, &MotorHandler::tweakCB, this);
	mTwistSub 		= mNodeHandle.subscribe("/movementTopic", 10, &MotorHandler::moveCB, this);
	mPositionSub 	= mNodeHandle.subscribe("/positionTopic", 10, &MotorHandler::positionCB, this);
	mUltrasoneSub 	= mNodeHandle.subscribe("/sensorFeedbackTopic", 10, &MotorHandler::ultrasoneCB, this);
	mDummySub		= mNodeHandle.subscribe("/dummyTopic", 10, &MotorHandler::dummyCB, this);
	//mDisableSub = mNodeHandle.subscribe("/disableMotorTopic", 10, &MotorHandler::disableMotorCB, this);

	mLock = false;
	mLeftMotor.init(path);
	mRightMotor.init(path);

	//Initialise distances from ultrasone sensors
	mRear = std::numeric_limits<int>::infinity();
	mFront = std::numeric_limits<int>::infinity();
	mRearLeft = std::numeric_limits<int>::infinity();
	mFrontLeft = std::numeric_limits<int>::infinity();
	mRearRight = std::numeric_limits<int>::infinity();
	mFrontRight = std::numeric_limits<int>::infinity();
	mRight = std::numeric_limits<int>::infinity();
	mLeft = std::numeric_limits<int>::infinity();

	//Activate all ultrasone sensors
	std_msgs::UInt8 activate_msg;
	activate_msg.data = ULTRASONE_ALL;
	mUltrasoneActivatePub.publish(activate_msg);

	ROS_INFO("Initialising completed.");
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "MotorHandler");

	char *path=NULL;
	if (argc == 2)
		path = argv[1];

	MotorHandler motorHandler;
	motorHandler.init(path);

	while(ros::ok())
	{
		motorHandler.publishRobotSpeed();
		ros::spinOnce();
	}

	return 0;
}
