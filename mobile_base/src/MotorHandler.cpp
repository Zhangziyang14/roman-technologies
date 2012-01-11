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
	double currentRightPosition = mRightMotor.getPosition();
	double currentLeftPosition = mLeftMotor.getPosition();

	mLock = true;
	mRightMotor.setMode(CM_POSITION_MODE);
	mLeftMotor.setMode(CM_POSITION_MODE);

	if((msg.data > 0 && mFrontLeftCenter > msg.data*100 && mFrontRightCenter > msg.data*100) || (msg.data < 0 && mRearLeft > std::abs(msg.data*100) && mRearRight > std::abs(msg.data*100)))
	{
		mRightMotor.setPosition(currentRightPosition + msg.data);
		mLeftMotor.setPosition(currentLeftPosition + msg.data);
	}
}

/**
 * Controls the speed of the motors based on the received twist message.
 */
void MotorHandler::moveCB(const geometry_msgs::Twist& msg)
{
	//Robot is boxed, so block the robot
	if(mLeft < 50 && mRight < 50 && mFrontLeftCenter < 80 && mFrontRightCenter < 80)
		mLock = true;
	//Unblock the robot
	else if(mCurrentSpeed.linear.x == 0)
		mLock = false;

	double converted_right	= ZERO_SPEED, converted_left = ZERO_SPEED;
	double left_speed 	 	= ZERO_SPEED;
	double right_speed 		= ZERO_SPEED;

	//Convert linear speed to motor speeds in [rad/s]
	if (left_speed == ZERO_SPEED && right_speed == ZERO_SPEED)
	{
		double vel_linear = msg.linear.x / WHEEL_RADIUS;
		double vel_angular = msg.angular.z * (BASE_RADIUS / WHEEL_RADIUS) * 2;

		converted_left  = vel_linear - 0.5*vel_angular;
		converted_right = vel_linear + 0.5*vel_angular;
	}
	if(mLock)
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
	else
	{
		//Calculations are in positive numbers because the minimum between speed and scaled speed is used
		left_speed 	= std::abs(converted_left);
		right_speed = std::abs(converted_right);

		left_speed      = scaleSpeed(left_speed, STANDARD_AVOIDANCE_IMPACT, FRONT_SIDES_AVOIDANCE_DISTANCE, mFrontRight, mFrontCenterRight);
		right_speed		= scaleSpeed(right_speed, STANDARD_AVOIDANCE_IMPACT, FRONT_SIDES_AVOIDANCE_DISTANCE, mFrontLeft, mFrontCenterLeft);

		//Check right and left side for walls and avoid them when needed.
		//Don't avoid the walls when distance to both sides are close to the robot
		if(!(mLeft < SIDES_AVOIDANCE_DISTANCE && mRight < SIDES_AVOIDANCE_DISTANCE))
		{
			left_speed	= scaleSpeed(left_speed, SIDES_AVOIDANCE_IMPACT, SIDES_AVOIDANCE_DISTANCE, mRight);
			right_speed	= scaleSpeed(right_speed, SIDES_AVOIDANCE_IMPACT, SIDES_AVOIDANCE_DISTANCE, mLeft);
		}
		//Avoidance rules when object is in front of robot
		if(mFrontLeftCenter < FRONT_AVOIDANCE_DISTANCE || mFrontRightCenter < FRONT_AVOIDANCE_DISTANCE)
		{
			//Turn to left when left side has more space than right side
			if(mLeft > mRight && mRight < SIDES_AVOIDANCE_DISTANCE)
				left_speed = scaleSpeed(left_speed, FRONT_AVOIDANCE_IMPACT, FRONT_AVOIDANCE_DISTANCE, mFrontLeftCenter, mFrontRightCenter);

			//Turn to right when right side has more space than left side
			else if(mLeft < mRight && mLeft < SIDES_AVOIDANCE_DISTANCE)
				right_speed = scaleSpeed(right_speed, FRONT_AVOIDANCE_IMPACT, FRONT_AVOIDANCE_DISTANCE, mFrontLeftCenter, mFrontRightCenter);

			//Turn right when object is more to the left of the robot
			else if(mFrontLeft < FRONT_AVOIDANCE_DISTANCE || mFrontCenterLeft < FRONT_AVOIDANCE_DISTANCE)
				right_speed = scaleSpeed(right_speed, FRONT_AVOIDANCE_IMPACT, FRONT_AVOIDANCE_DISTANCE, mFrontLeftCenter, mFrontRightCenter);

			//Turn left when object is more to the right of the robot
			else if(mFrontRight < FRONT_AVOIDANCE_DISTANCE || mFrontCenterRight < FRONT_AVOIDANCE_DISTANCE)
				left_speed = scaleSpeed(left_speed, FRONT_AVOIDANCE_IMPACT, FRONT_AVOIDANCE_DISTANCE, mFrontLeftCenter, mFrontRightCenter);

			//Turn right when robot is driving straight to a wall
			else if(mFrontCenterRight < FRONT_SIDES_AVOIDANCE_DISTANCE-20 && mFrontCenterLeft < FRONT_SIDES_AVOIDANCE_DISTANCE-20)
			{
				right_speed = scaleSpeed(right_speed, 0.5*FRONT_CENTER_AND_SIDES_AVOIDANCE_IMPACT, FRONT_SIDES_AVOIDANCE_DISTANCE, mFrontLeftCenter, mFrontRightCenter);
			}
			else
			{
				//If there is space front right of the robot, then turn to that side. Otherwise, turn to the other side.
				if(mFrontRight > FRONT_AVOIDANCE_DISTANCE)
					right_speed = scaleSpeed(right_speed, FRONT_CENTER_AND_SIDES_AVOIDANCE_IMPACT, FRONT_AVOIDANCE_DISTANCE, mFrontLeftCenter, mFrontRightCenter);
				else
					left_speed = scaleSpeed(right_speed, FRONT_CENTER_AND_SIDES_AVOIDANCE_IMPACT, FRONT_AVOIDANCE_DISTANCE, mFrontLeftCenter, mFrontRightCenter);
			}
		}

		//Restore the direction of the speeds when all calculations are done
		left_speed = converted_left < ZERO_SPEED ? -left_speed : left_speed;
		right_speed = converted_right < ZERO_SPEED ? -right_speed : right_speed;

		//Don't go backwards when there is a wall
		if(msg.linear.x < 0 && mRearLeft < REAR_HALTING_DISTANCE && mRearRight < REAR_HALTING_DISTANCE)
		{
			left_speed = 0.0;
			right_speed = 0.0;
		}

		//ROS_INFO("left %f right %f", left_speed, right_speed);

		mRightMotor.setSpeed(right_speed);
		mLeftMotor.setSpeed(left_speed);
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

void MotorHandler::negateUltrasone(int length, int* ultrasone[])
{
	for(int i = 0; i < length ; i++)
		*ultrasone[i] = 150;
}

/**
 *	Reads distances from ultrasone sensors
 */
void MotorHandler::ultrasoneCB(const mobile_base::sensorFeedback& msg)
{
	if(mCurrentSpeed.linear.x > 0)
	{
		int* ultrasone[2] = {&mRearLeft, &mRearRight};
		negateUltrasone(2, ultrasone);

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
	mUltrasoneSub 	= mNodeHandle.subscribe("/sensorFeedbackTopic", 10, &MotorHandler::ultrasoneCB, this);

	mLock = false;

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

