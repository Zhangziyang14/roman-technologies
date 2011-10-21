#include <MotorHandler.h>
/*
void MotorHandler::checkConnections()
{
	if(mTweakPIDSub.getNumPublishers() == 0 || mTwistSub.getNumPublishers() == 0)
	{
		if(mTweakPIDSub.getNumPublishers() == 0)
			ROS_INFO("%s has died", mTweakPIDSub.getTopic().c_str());
		else
			ROS_INFO("%s has died", mTwistSub.getTopic().c_str());

		mController.killNode();
8	}
 */

/**
 * Publishes the linear and angular speed of the robot
 */
void MotorHandler::publishRobotSpeed()
{
	mRightMotorSpeed = mRightMotor.getRotationSpeed();
	mLeftMotorSpeed = mLeftMotor.getRotationSpeed();

	// linear speed is the average of both motor speeds and it is converted from rad/s to m/s
	mCurrentSpeed.linear.x  = (mRightMotorSpeed + mLeftMotorSpeed) * WHEEL_RADIUS / 2.0;
	mCurrentSpeed.angular.z = (mRightMotorSpeed - mLeftMotorSpeed) * getBaseRadius();

	mSpeedPub.publish(mCurrentSpeed);
}

/**
 * Check whether the Motors are still alive and re-initialise them if not
 */
void MotorHandler::checkMotorConnections(char* path)
{
	if(mLeftMotor.checkPort())
	{
		mLeftMotor.init(path);
	}

	if(mRightMotor.checkPort())
	{
		mRightMotor.init(path);
	}
}

/**
 * Controls the speed of the motors based on the received twist message.
 */
void MotorHandler::moveCB(const mobile_base::BaseMotorControl& msg)
{
	double left_vel  = msg.left_motor_speed ? msg.left_motor_speed / WHEEL_RADIUS : (msg.twist.linear.x  - msg.twist.angular.z * BASE_RADIUS) / WHEEL_RADIUS;
	double right_vel = msg.right_motor_speed ? msg.right_motor_speed / WHEEL_RADIUS : (msg.twist.linear.x  + msg.twist.angular.z* BASE_RADIUS) / WHEEL_RADIUS;

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

	mRightMotor.setSpeed(right_vel);
	mLeftMotor.setSpeed(left_vel);
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
 */
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

	/*if (mDisableForwardMotion)
		ROS_INFO("Disable forward.");
	if (mDisableBackwardMotion)
		ROS_INFO("Disable backward.");*/
}

/**
 * Initialise MotorHandler and its attributes.
 */
void MotorHandler::init(char *path)
{
	mSpeedPub = mNodeHandle.advertise<geometry_msgs::Twist>("/speedFeedbackTopic", 1);

	mTweakPIDSub = mNodeHandle.subscribe("/tweakTopic", 10, &MotorHandler::tweakCB, this);
	mTwistSub = mNodeHandle.subscribe("/movementTopic", 10, &MotorHandler::moveCB, this);
	mDisableSub = mNodeHandle.subscribe("/disableMotorTopic", 10, &MotorHandler::disableMotorCB, this);

	mLeftMotor.init(path);
	mRightMotor.init(path);

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
