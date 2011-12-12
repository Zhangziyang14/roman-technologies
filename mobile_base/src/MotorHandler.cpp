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
void MotorHandler::positionCB(const std_msgs::Float64& msg)
{
	//double currentRightPosition = mRightMotor.getPosition();
	//double currentLeftPosition = mLeftMotor.getPosition();

	//mLock = true;
	mRightMotor.setMode(CM_POSITION_MODE);
	mLeftMotor.setMode(CM_POSITION_MODE);

	/*if((msg.data > 0 && mFrontLeftCenter > msg.data*100 && mFrontRightCenter > msg.data*100) || (msg.data < 0 && mRearLeft > std::abs(msg.data*100) && mRearRight > std::abs(msg.data*100)))
	{
		mRightMotor.setPosition(currentRightPosition + msg.data);
		mLeftMotor.setPosition(currentLeftPosition + msg.data);
	}*/
}

/**
 * Controls the speed of the motors based on the received twist message.
 */
void MotorHandler::moveCB(const geometry_msgs::Twist& msg)
{
	double converted_right 	= ZERO_SPEED, converted_left = ZERO_SPEED;

	//Convert linear speed to motor speeds in [rad/s]
	double vel_linear = msg.linear.x / WHEEL_RADIUS;
	double vel_angular = msg.angular.z / (BASE_RADIUS / WHEEL_RADIUS);

	converted_left  = vel_linear - vel_angular;
	converted_right = vel_linear + vel_angular;
	/*if(mLock)
	{
		//Disable motors when boxed or when in position mode
		if((mCurrentSpeed.linear.x > ZERO_SPEED && (mFrontLeftCenter < 80 || mFrontRightCenter < 80)) ||  (mCurrentSpeed.linear.x < ZERO_SPEED && (mRearLeft < 50 || mRearRight < 50)))
		{
			mRightMotor.brake();
			mLeftMotor.brake();

			left_speed	= ZERO_SPEED;
			right_speed	= ZERO_SPEED;
		}
	}
	else*/
	{

		mRightMotor.setSpeed(converted_right);
		mLeftMotor.setSpeed(converted_left);
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
 * Initialise MotorHandler and its attributes.
 */
void MotorHandler::init(char *path)
{
	//Initialise publishers
	mSpeedPub = mNodeHandle.advertise<geometry_msgs::Twist>("/speedFeedbackTopic", 1);
	mUltrasoneActivatePub = mNodeHandle.advertise<std_msgs::Bool>("/sensorActivateTopic", 10);

	//Initialise subscribers
	mTweakPIDSub 	= mNodeHandle.subscribe("/tweakTopic", 10, &MotorHandler::tweakCB, this);
	mTwistSub 		= mNodeHandle.subscribe("/cmd_vel", 10, &MotorHandler::moveCB, this);
	mPositionSub 	= mNodeHandle.subscribe("/positionTopic", 10, &MotorHandler::positionCB, this);

	mLock = false;
	mLeftMotor.init(path);
	mRightMotor.init(path);

	//Activate all ultrasone sensors
	std_msgs::Bool activate_msg;
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

