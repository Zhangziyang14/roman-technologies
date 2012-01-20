/*
 * LocalPlanner.cpp
 *
 *  Created on: 2012-01-20
 *      Author: wilson
 */
#include <mobile_base/LocalPlanner.h>

LocalPlanner::LocalPlanner(ros::NodeHandle *nodeHandle)
{
	//Initialise Publishers
	mSpeedPub = nodeHandle->advertise<geometry_msgs::Twist>("cmd_vel", 1);

	//Initialise Subscribers
	mUltrasoneSub = nodeHandle->subscribe("/sensorFeedbackTopic", 10, &LocalPlanner::ultrasoneCB, this);
	mSpeedSub	  = nodeHandle->subscribe("speedFeedbackTopic", 10, &LocalPlanner::speedCB, this);

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

}

/**
 * Listens to current speed of the mobile base
 */
void LocalPlanner::speedCB(const geometry_msgs::Twist &msg)
{
	mCurrentSpeed = msg;
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
void LocalPlanner::ultrasoneCB(const mobile_base::sensorFeedback& msg)
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
 * Scales twist message and publishes the scaled message
 */
void LocalPlanner::scaleTwist(geometry_msgs::Twist& msg)
{
	double converted_right	= ZERO_SPEED, converted_left = ZERO_SPEED;
	double left_speed 	 	= ZERO_SPEED;
	double right_speed 		= ZERO_SPEED;

	//Convert linear speed to motor speeds in [rad/s]
	double vel_linear = msg.linear.x / WHEEL_RADIUS;
	double vel_angular = msg.angular.z * (BASE_RADIUS / WHEEL_RADIUS) * 2;

	converted_left  = vel_linear - 0.5*vel_angular;
	converted_right = vel_linear + 0.5*vel_angular;

	//Disable motors when boxed or when in position mode
	if((mCurrentSpeed.linear.x > ZERO_SPEED && (mFrontLeftCenter < 80 || mFrontRightCenter < 80)) ||  (mCurrentSpeed.linear.x < ZERO_SPEED && (mRearLeft < 50 || mRearRight < 50)))
	{
		msg.angular.z 	= 0;
		msg.linear.x 	= 0;
		return;
	}

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
		msg.linear.x = 0;
		msg.angular.z = 0;
		return;
	}

	//ROS_INFO("left %f right %f", left_speed, right_speed);


	// linear speed is the average of both motor speeds and it is converted from rad/s to m/s
	msg.linear.x  = (right_speed + left_speed) * WHEEL_RADIUS / 2.0;
	msg.angular.z = (right_speed - left_speed) * WHEEL_RADIUS / 0.25 /2.0; //* getBaseRadius();
}


