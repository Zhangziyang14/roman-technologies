#include <MotorHandler.h>

/**
 * Publishes the linear and angular speed of the robot
 */
void MotorHandler::publishRobotSpeed()
{
	mRightMotorSpeed = mRightMotor.getRotationSpeed();
	mLeftMotorSpeed = mLeftMotor.getRotationSpeed();

	mCurrentSpeed.linear.x  = (mRightMotorSpeed + mLeftMotorSpeed) * WHEEL_RADIUS / 2.0;
	mCurrentSpeed.angular.z = (mRightMotorSpeed - mLeftMotorSpeed) * getBaseRadius();

	mSpeedSub.publish(mCurrentSpeed);
}

/**
 * Controls the speed of the motors based on the received twist message.
 */
void MotorHandler::moveCB(const mobile_base::BaseMotorControl& msg)
{
	double left_vel  = msg.left_motor_speed ? msg.left_motor_speed / WHEEL_RADIUS : (msg.twist.linear.x  - msg.twist.angular.z * BASE_RADIUS) / WHEEL_RADIUS;
	double right_vel = msg.right_motor_speed ? msg.right_motor_speed / WHEEL_RADIUS : (msg.twist.linear.x  + msg.twist.angular.z* BASE_RADIUS) / WHEEL_RADIUS;

	mRightMotor.setSpeed(right_vel);
	mLeftMotor.setSpeed(left_vel);
}

/**
 * Called when a Twist message is received over the motor topic.
 */
void MotorHandler::tweakCB(const mobile_base::tweak msg)
{
	Motor *motor = mMotorId == MID_LEFT ? &mLeftMotor : &mRightMotor;

	switch (msg.data)
	{
	case PS3_UP:
	case PS3_DOWN:
		motor->mPID[mPIDFocus] += msg.data == PS3_DOWN ? -PID_TWEAK_STEP : PID_TWEAK_STEP;
		motor->mPID[mPIDFocus] = std::max(0.0, motor->mPID[mPIDFocus]);
		motor->updatePID();
		motor->printPID();
		break;

	case PS3_LEFT:
	case PS3_RIGHT:
		// toggle through P-I-D
		mPIDFocus = PIDParameter((mPIDFocus + (msg.data == PS3_LEFT ? -1 : 1)) % PID_PARAM_MAX);

		switch (mPIDFocus)
		{
		case PID_PARAM_P: ROS_INFO("P"); break;
		case PID_PARAM_I: ROS_INFO("I"); break;
		case PID_PARAM_D: ROS_INFO("D"); break;
		default: break;
		}

		break;

		case PS3_L1:
		case PS3_R1:
			mMotorId = MotorId(msg.motorID);
			ROS_INFO("SWITCHED TO %s ENGINE", mMotorId == MID_LEFT ? "LEFT" : "RIGHT");
			break;
	}
}

/**
 * Initialise MotorHandler and its attributes.
 */
void MotorHandler::init(char *path)
{
	mSpeedSub = mNodeHandle.advertise<geometry_msgs::Twist>("/speedFeedbackTopic", 1);
	mTweakPIDSub = mNodeHandle.subscribe("/tweakTopic", 10, &MotorHandler::tweakCB, this);
	mTwistSub = mNodeHandle.subscribe("/movementTopic", 10, &MotorHandler::moveCB, this);
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
