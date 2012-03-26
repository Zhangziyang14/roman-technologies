#include <mobile_base/MotorHandler.h>

using namespace nero_msgs;

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
 * Is not used because arduino does not support any more service servers or subscribers...

void MotorHandler::disableUltrasoneSensors()
{
	mobile_base::disableUltrasone disable_srv;
	disable_srv.request.disable = mCurrentSpeed.linear.x > 0 ? uint16_t(SensorFeedback::DISABLE_REAR) : uint16_t(SensorFeedback::DISABLE_FRONT);

	if(mCurrentSpeed.linear.x == 0)
		disable_srv.request.disable = uint16_t(SensorFeedback::DISABLE_NONE);

	if(mCurrentUltrasoneState != disable_srv.request.disable)
		mUltrasoneDisableClient.call(disable_srv);

	mCurrentUltrasoneState = disable_srv.request.disable;
}
**/

/**
 * Controls the motors based on the received position.
 */
void MotorHandler::positionCB(const nero_msgs::MotorPosition& msg)
{
	mRightMotor.setSpeed(0.0);
	mLeftMotor.setSpeed(0.0);

	double currentRightPosition = mRightMotor.getPosition();
	double currentLeftPosition = mLeftMotor.getPosition();

	mLock = true;

	//ROS_INFO("left: %f, right: %f, front_l: %d, front_r: %d, rear_l: %d, rear_r: %d", msg.left, msg.right, mFrontLeftCenter, mFrontRightCenter, mRearLeft, mRearRight);
		mRightMotor.setPosition(currentRightPosition + msg.right);
		mLeftMotor.setPosition(currentLeftPosition + msg.left);
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
 *	Reads distances from ultrasone sensors
 */
void MotorHandler::ultrasoneCB(const nero_msgs::SensorFeedback& msg)
{
	mSensorData = msg.data;
	//ROS_INFO("left %d, frontLeft %d, frontCenterLeft %d, frontLeftCenter %d, frontRightCenter %d, frontCenterRight %d, right %d", mLeft, mFrontLeft, mFrontCenterLeft, mFrontLeftCenter, mFrontRightCenter, mFrontCenterRight, mRight);
	//ROS_INFO("rearLeft %d, rearRight %d", mRearLeft, mRearRight);
}

/**
 * Stops all motors and locks them until unlock command is received
 */
void MotorHandler::stopCB(const std_msgs::Bool &msg)
{
	if(msg.data)
	{
		mRightMotor.setMode(CM_STOP_MODE);
		mLeftMotor.setMode(CM_STOP_MODE);
	}

	mRightMotor.lock(msg.data);
	mLeftMotor.lock(msg.data);
}

/**
 * Initialise MotorHandler and its attributes.
 */
void MotorHandler::init(char *path)
{
	//Initialise publishers
	//mDisableUltrasonePub 	= mNodeHandle.advertise<mobile_base::SensorFeedback>("/sensorDisableTopic", 1);
	mSpeedPub 				= mNodeHandle.advertise<geometry_msgs::Twist>("/speedFeedbackTopic", 1);

	//Initialise subscribers
	mTwistSub 				= mNodeHandle.subscribe("/cmd_vel", 10, &MotorHandler::moveCB, this);
	mPositionSub 			= mNodeHandle.subscribe("/positionTopic", 10, &MotorHandler::positionCB, this);
	mUltrasoneSub   		= mNodeHandle.subscribe("/sensorFeedbackTopic", 10, &MotorHandler::ultrasoneCB, this);
	mStopSubscriber			= mNodeHandle.subscribe("/emergencyStop", 1, &MotorHandler::stopCB, this);

	//Initialise service clients
	//mUltrasoneDisableClient = mNodeHandle.serviceClient<mobile_base::disableUltrasone>("/disableUltrasoneService", true);

	mLock = false;

	mLeftMotor.init(path);
	mRightMotor.init(path);

	mLeftMotor.setAcceleration(DEFAULT_ACCELERATION);
	mRightMotor.setAcceleration(DEFAULT_ACCELERATION);

	//Initialise distances from ultrasone sensors
	for (int sensor = 0; sensor < SensorFeedback::SENSOR_COUNT; sensor++)
		mSensorData[sensor] = SensorFeedback::ULTRASONE_MAX_RANGE;

	//mCurrentUltrasoneState = 0;

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
		//motorHandler.disableUltrasoneSensors();

		ros::spinOnce();
		sleep.sleep();
	}

	return 0;
}
