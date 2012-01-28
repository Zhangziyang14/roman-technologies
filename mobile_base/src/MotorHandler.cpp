#include <mobile_base/MotorHandler.h>

using namespace mobile_base;

/**
 * Publishes the linear and angular speed of the robot
 */
void MotorHandler::publishRobotSpeed()
{
	mRightMotorSpeed = mRightMotor.getRotationSpeed();
	mLeftMotorSpeed	 = mLeftMotor.getRotationSpeed();

	// linear speed is the average of both motor speeds and it is converted from rad/s to m/s
	mCurrentSpeed.linear.x  = (mRightMotorSpeed + mLeftMotorSpeed) * WHEEL_RADIUS / 2.0;
	mCurrentSpeed.angular.z = (mRightMotorSpeed - mLeftMotorSpeed) * WHEEL_RADIUS / 0.25 /2.0; //* getBaseRadius();

	mSpeedPub.publish(mCurrentSpeed);
}

/**
 * Disables unused ultrasone sensors
 */
void MotorHandler::disableUltrasoneSensors()
{
	mobile_base::SensorFeedback disable_msg;

	//Disable rear sensors and activate front sensors when driving forward
	if(mCurrentSpeed.linear.x > 0)
	{
		for(int sensor = 0; sensor <= SensorFeedback::SENSOR_FRONT_RIGHT; sensor++)
			disable_msg.data[sensor] = DISABLE_FALSE;

		disable_msg.data[SensorFeedback::SENSOR_REAR_RIGHT] = DISABLE_TRUE;
		disable_msg.data[SensorFeedback::SENSOR_REAR_LEFT] = DISABLE_TRUE;
	}
	//Disable rear sensors and activate front sensors when driving backwards
	else if(mCurrentSpeed.linear.x < 0)
	{
		for(int sensor = 0; sensor <= SensorFeedback::SENSOR_FRONT_RIGHT; sensor++)
			disable_msg.data[sensor] = DISABLE_FALSE;

		disable_msg.data[SensorFeedback::SENSOR_REAR_RIGHT] = DISABLE_TRUE;
		disable_msg.data[SensorFeedback::SENSOR_REAR_LEFT] = DISABLE_TRUE;
	}
	//Activate all sensors when standing still
	else
	{
		for(int sensor = 0; sensor <= SensorFeedback::SENSOR_COUNT; sensor++)
			disable_msg.data[sensor] = DISABLE_FALSE;
	}
	mDisableUltrasonePub.publish(disable_msg);
}

/**
 * Controls the motors based on the received position.
 */
void MotorHandler::positionCB(const mobile_base::position& msg)
{
	mRightMotor.setSpeed(0.0);
	mLeftMotor.setSpeed(0.0);

	double currentRightPosition = mRightMotor.getPosition();
	double currentLeftPosition = mLeftMotor.getPosition();

	mLock = true;

	//ROS_INFO("left: %f, right: %f, front_l: %d, front_r: %d, rear_l: %d, rear_r: %d", msg.left, msg.right, mFrontLeftCenter, mFrontRightCenter, mRearLeft, mRearRight);
	if((msg.left > 0 && msg.right > 0  && mSensorData[SensorFeedback::SENSOR_FRONT_CENTER_LEFT] > msg.left*100 && mSensorData[SensorFeedback::SENSOR_FRONT_CENTER_RIGHT] > msg.left*100) ||
			(msg.left < 0 && msg.right < 0 && mSensorData[SensorFeedback::SENSOR_REAR_LEFT] > std::abs(msg.left*100) && mSensorData[SensorFeedback::SENSOR_REAR_RIGHT] > std::abs(msg.left*100)) ||
			msg.left != msg.right)
	{
		mRightMotor.setPosition(currentRightPosition + msg.right);
		mLeftMotor.setPosition(currentLeftPosition + msg.left);
	}
	else if(msg.left == 0 && msg.right == 0)
	{
		mRightMotor.stopAtPosition(currentRightPosition);
		mLeftMotor.stopAtPosition(currentRightPosition);
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
 *	Reads distances from ultrasone sensors
 */
//TODO: Turn off the unused ultrasone sensors in arduino code
void MotorHandler::ultrasoneCB(const mobile_base::SensorFeedback& msg)
{
	mSensorData = msg.data;
	//ROS_INFO("left %d, frontLeft %d, frontCenterLeft %d, frontLeftCenter %d, frontRightCenter %d, frontCenterRight %d, right %d", mLeft, mFrontLeft, mFrontCenterLeft, mFrontLeftCenter, mFrontRightCenter, mFrontCenterRight, mRight);
	//ROS_INFO("rearLeft %d, rearRight %d", mRearLeft, mRearRight);
}

/**
 * Initialise MotorHandler and its attributes.
 */
void MotorHandler::init(char *path)
{
	//Initialise publishers
	mDisableUltrasonePub 	= mNodeHandle.advertise<mobile_base::SensorFeedback>("/sensorDisableTopic", 1);
	mSpeedPub 				= mNodeHandle.advertise<geometry_msgs::Twist>("/speedFeedbackTopic", 1);

	//Initialise subscribers
	mTweakPIDSub 			= mNodeHandle.subscribe("/tweakTopic", 10, &MotorHandler::tweakCB, this);
	mTwistSub 				= mNodeHandle.subscribe("/cmd_vel", 10, &MotorHandler::moveCB, this);
	mPositionSub 			= mNodeHandle.subscribe("/positionTopic", 10, &MotorHandler::positionCB, this);
	mUltrasoneSub   		= mNodeHandle.subscribe("/sensorFeedbackTopic", 10, &MotorHandler::ultrasoneCB, this);

	mLock = false;

	mLeftMotor.init(path);
	mRightMotor.init(path);

	//Initialise distances from ultrasone sensors
	for (int i = 0; i < SensorFeedback::SENSOR_COUNT; i++)
		mSensorData[i] = SensorFeedback::ULTRASONE_MAX_RANGE;

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

	int sleep_rate;
	motorHandler.getNodeHandle()->param<int>("node_sleep_rate", sleep_rate, 50);
	ros::Rate sleep(sleep_rate);

	while(ros::ok())
	{
		motorHandler.publishRobotSpeed();
		ros::spinOnce();
		sleep.sleep();
	}

	return 0;
}
