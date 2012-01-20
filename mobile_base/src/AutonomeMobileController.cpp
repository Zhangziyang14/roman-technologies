/*
 * AutonomeMobileController.cpp
 *
 *  Created on: 2012-01-20
 *      Author: wilson
 */
#include "mobile_base/AutonomeMobileController.h"

/**
 * Controls the speed of the motors based on the received twist message.
 */
void AutonomeMobileController::scaleTwist(const geometry_msgs::Twist& msg)
{
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

/**
 * Main follow loop.
 */
void AutonomeMobileController::spin()
{
	ros::Rate refreshRate(mRefreshRate);

	while (ros::ok())
	{
		mPathFollower.updatePath();

		refreshRate.sleep();
		ros::spinOnce();
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "AutonomeMobileController");

	AutonomeMobileController autonomeMobileController;

	autonomeMobileController.spin();
	return 0;
}
