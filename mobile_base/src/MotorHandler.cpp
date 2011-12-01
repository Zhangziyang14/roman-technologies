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

	if((msg.data > 0 && mFrontLeftCenter > msg.data*100 && mFrontRightCenter > msg.data*100) || (msg.data < 0 && mRearLeft > std::abs(msg.data*100) && mRearRight > std::abs(msg.data*100)))
	{
		mRightMotor.setPosition(currentRightPosition + msg.data);
		mLeftMotor.setPosition(currentLeftPosition + msg.data);
	}
}

/**
 * Controls the speed of the motors based on the received twist message.
 */
void MotorHandler::moveCB(const mobile_base::BaseMotorControl& msg)
{
	if(mLeft < 50 && mRight < 50 && mFrontLeftCenter < 80 && mFrontRightCenter < 80)
		mLock = true;
	else if(mCurrentSpeed.linear.x == 0)
		mLock = false;
	
	double converted_right = 0, converted_left = 0;
	double left_speed  	= msg.left_motor_speed;
	double right_speed 	= msg.right_motor_speed;

	if (left_speed == 0.0 && right_speed == 0.0)
	{
		double vel_linear = msg.twist.linear.x / WHEEL_RADIUS;
		double vel_angular = msg.twist.angular.z / (BASE_RADIUS / WHEEL_RADIUS);

		converted_left  = vel_linear - vel_angular;
		converted_right = vel_linear + vel_angular;
	}
	if(mLock)
	{
		//Safe positioning while bumping
		if((mCurrentSpeed.linear.x > 0 && (mFrontLeftCenter < 80 || mFrontRightCenter < 80)) ||  (mCurrentSpeed.linear.x < 0 && (mRearLeft < 50 || mRearRight < 50)))
		{
			mRightMotor.brake();
			mLeftMotor.brake();
			left_speed = 0.0;
			right_speed = 0.0;
		}
	}
	else
	{
	left_speed = std::abs(converted_left);
	right_speed = std::abs(converted_right);

	left_speed      	= std::min(left_speed-left_speed*(120.0-mFrontRight)/175.0,left_speed);
	right_speed  	        = std::min(right_speed-right_speed*(120.0-mFrontLeft)/175.0,right_speed);

		//check right and left side for walls
		if(!(mLeft < 60 && mRight < 60))
		{
			left_speed	= std::min(left_speed-left_speed*(60.0-mRight)/75.0, left_speed);
			right_speed	= std::min(right_speed-right_speed*(60.0-mLeft)/75.0, right_speed);
		}
		if(mFrontLeftCenter < 150 || mFrontRightCenter < 150)
		{

			if(mLeft > mRight && mRight < 75)
				left_speed = std::min(left_speed, std::min(left_speed-left_speed*(150.0-mFrontLeftCenter)/100.0, left_speed-left_speed*(150.0-mFrontRightCenter)/100.0));

			else if(mLeft < mRight && mLeft < 75)
				right_speed = std::min(right_speed, std::min(right_speed-right_speed*(150.0-mFrontLeftCenter)/100.0, right_speed-right_speed*(150.0-mFrontRightCenter)/100.0));

			else if(mFrontLeft < 150)
				right_speed = std::min(right_speed, std::min(right_speed-right_speed*(150.0-mFrontLeftCenter)/100.0, right_speed-right_speed*(150.0-mFrontRightCenter)/100.0));

			else if(mFrontRight < 150)
				left_speed = std::min(left_speed, std::min(left_speed-left_speed*(150.0-mFrontLeftCenter)/100.0, left_speed-left_speed*(150.0-mFrontRightCenter)/100.0));

			else if(mFrontRight < 100 && mFrontLeft < 100)
				right_speed = std::min(right_speed, std::min(right_speed-right_speed*(120.0-mFrontLeftCenter)/60.0, right_speed-right_speed*(120.0-mFrontRightCenter)/60.0));

			else
			{
				if(mFrontRight > 150)
					right_speed = std::min(right_speed, std::min(right_speed-right_speed*(150.0-mFrontLeftCenter)/120.0, right_speed-right_speed*(150.0-mFrontRightCenter)/120.0));
				else
					left_speed = std::min(right_speed, std::min(right_speed-right_speed*(150.0-mFrontLeftCenter)/120.0, right_speed-right_speed*(150.0-mFrontRightCenter)/120.0));
			}
		}

		left_speed = converted_left < 0 ? -left_speed : left_speed;
		right_speed = converted_right < 0 ? -right_speed : right_speed;
		
		ROS_INFO("left %f right %f", left_speed, right_speed);

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

/**
 *	Reads distances from ultrasone sensors
 */
void MotorHandler::ultrasoneCB(const mobile_base::sensorFeedback& msg)
{
	mFrontLeftCenter	= msg.frontLeftCenter;
	mFrontRightCenter	= msg.frontRightCenter;
	mFrontLeft 		= msg.frontLeft;
	mFrontRight		= msg.frontRight;

	mRearLeft		= msg.rearLeft;
	mRearRight		= msg.rearRight;

	mLeft			= msg.left;
	mRight			= msg.right;
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
	mTwistSub 	= mNodeHandle.subscribe("/movementTopic", 10, &MotorHandler::moveCB, this);
	mPositionSub 	= mNodeHandle.subscribe("/positionTopic", 10, &MotorHandler::positionCB, this);
	mUltrasoneSub 	= mNodeHandle.subscribe("/sensorFeedbackTopic", 10, &MotorHandler::ultrasoneCB, this);

	mLock = false;
	mLeftMotor.init(path);
	mRightMotor.init(path);

	//Initialise distances from ultrasone sensors
	mFrontLeftCenter = ULTRASONE_MAX_RANGE; 
	mFrontRightCenter = ULTRASONE_MAX_RANGE;
	mRearLeft = ULTRASONE_MAX_RANGE;
	mFrontLeft = ULTRASONE_MAX_RANGE;
	mRearRight = ULTRASONE_MAX_RANGE;
	mFrontRight = ULTRASONE_MAX_RANGE;
	mRight = ULTRASONE_MAX_RANGE;
	mLeft = ULTRASONE_MAX_RANGE;

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

