/*
 * LocalPlanner.cpp
 *
 *  Created on: 2012-01-20
 *      Author: wilson
 */
#include <mobile_base/LocalPlanner.h>

using namespace mobile_base;

LocalPlanner::LocalPlanner(ros::NodeHandle *nodeHandle)
{
	//Initialise Publishers
	mSpeedPub = nodeHandle->advertise<geometry_msgs::Twist>("cmd_vel", 1);

	//Initialise Subscribers
	mUltrasoneSub = nodeHandle->subscribe("/sensorFeedbackTopic", 10, &LocalPlanner::ultrasoneCB, this);
	mSpeedSub	  = nodeHandle->subscribe("speedFeedbackTopic", 10, &LocalPlanner::speedCB, this);

	//Initialise distances from ultrasone sensors
	for (int i = 0; i < SensorFeedback::SENSOR_COUNT; i++)
		mSensorData[i] = SensorFeedback::ULTRASONE_MAX_RANGE;

}

/**
 * Listens to current speed of the mobile base
 */
void LocalPlanner::speedCB(const geometry_msgs::Twist &msg)
{
	mCurrentSpeed = msg;
}

/**s
 *	Reads distances from ultrasone sensors
 */
void LocalPlanner::ultrasoneCB(const mobile_base::SensorFeedback& msg)
{
	mSensorData = msg.data;

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
	/*if((mCurrentSpeed.linear.x > ZERO_SPEED && (mFrontLeftCenter < 80 || mFrontRightCenter < 80)) ||  (mCurrentSpeed.linear.x < ZERO_SPEED && (mRearLeft < 50 || mRearRight < 50)))
	{
		msg.angular.z 	= 0;
		msg.linear.x 	= 0;
		return;
	}*/

	//Calculations are in positive numbers because the minimum between speed and scaled speed is used
	left_speed 	= std::abs(converted_left);
	right_speed = std::abs(converted_right);

	left_speed      = scaleSpeed(left_speed, STANDARD_AVOIDANCE_IMPACT, FRONT_SIDES_AVOIDANCE_DISTANCE, mSensorData[SensorFeedback::SENSOR_FRONT_RIGHT], mSensorData[SensorFeedback::SENSOR_FRONT_RIGHT_CENTER]);
	right_speed		= scaleSpeed(right_speed, STANDARD_AVOIDANCE_IMPACT, FRONT_SIDES_AVOIDANCE_DISTANCE, mSensorData[SensorFeedback::SENSOR_FRONT_LEFT], mSensorData[SensorFeedback::SENSOR_FRONT_LEFT_CENTER]);

	//Check right and left side for walls and avoid them when needed.
	//Don't avoid the walls when distance to both sides are close to the robot
	if(!(mSensorData[SensorFeedback::SENSOR_LEFT] < SIDES_AVOIDANCE_DISTANCE && mSensorData[SensorFeedback::SENSOR_RIGHT] < SIDES_AVOIDANCE_DISTANCE))
	{
		left_speed	= scaleSpeed(left_speed, SIDES_AVOIDANCE_IMPACT, SIDES_AVOIDANCE_DISTANCE, mSensorData[SensorFeedback::SENSOR_RIGHT]);
		right_speed	= scaleSpeed(right_speed, SIDES_AVOIDANCE_IMPACT, SIDES_AVOIDANCE_DISTANCE, mSensorData[SensorFeedback::SENSOR_LEFT]);
	}
	//Avoidance rules when object is in front of robot
	if(mSensorData[SensorFeedback::SENSOR_FRONT_CENTER_LEFT] < FRONT_AVOIDANCE_DISTANCE || mSensorData[SensorFeedback::SENSOR_FRONT_CENTER_RIGHT] < FRONT_AVOIDANCE_DISTANCE)
	{
		//Turn to left when left side has more space than right side
		if(mSensorData[SensorFeedback::SENSOR_LEFT] > mSensorData[SensorFeedback::SENSOR_RIGHT] && mSensorData[SensorFeedback::SENSOR_RIGHT] < SIDES_AVOIDANCE_DISTANCE)
			left_speed = scaleSpeed(left_speed, FRONT_AVOIDANCE_IMPACT, FRONT_AVOIDANCE_DISTANCE, mSensorData[SensorFeedback::SENSOR_FRONT_CENTER_LEFT], mSensorData[SensorFeedback::SENSOR_FRONT_CENTER_RIGHT]);

		//Turn to right when right side has more space than left side
		else if(mSensorData[SensorFeedback::SENSOR_LEFT] < mSensorData[SensorFeedback::SENSOR_RIGHT] && mSensorData[SensorFeedback::SENSOR_LEFT] < SIDES_AVOIDANCE_DISTANCE)
			right_speed = scaleSpeed(right_speed, FRONT_AVOIDANCE_IMPACT, FRONT_AVOIDANCE_DISTANCE, mSensorData[SensorFeedback::SENSOR_FRONT_CENTER_LEFT], mSensorData[SensorFeedback::SENSOR_FRONT_CENTER_RIGHT]);

		//Turn right when object is more to the left of the robot
		else if(mSensorData[SensorFeedback::SENSOR_FRONT_LEFT] < FRONT_AVOIDANCE_DISTANCE || mSensorData[SensorFeedback::SENSOR_FRONT_LEFT_CENTER] < FRONT_AVOIDANCE_DISTANCE)
			right_speed = scaleSpeed(right_speed, FRONT_AVOIDANCE_IMPACT, FRONT_AVOIDANCE_DISTANCE, mSensorData[SensorFeedback::SENSOR_FRONT_CENTER_LEFT], mSensorData[SensorFeedback::SENSOR_FRONT_CENTER_RIGHT]);

		//Turn left when object is more to the right of the robot
		else if(mSensorData[SensorFeedback::SENSOR_FRONT_RIGHT] < FRONT_AVOIDANCE_DISTANCE || mSensorData[SensorFeedback::SENSOR_FRONT_RIGHT_CENTER] < FRONT_AVOIDANCE_DISTANCE)
			left_speed = scaleSpeed(left_speed, FRONT_AVOIDANCE_IMPACT, FRONT_AVOIDANCE_DISTANCE, mSensorData[SensorFeedback::SENSOR_FRONT_CENTER_LEFT], mSensorData[SensorFeedback::SENSOR_FRONT_CENTER_RIGHT]);

		//Turn right when robot is driving straight to a wall
		else if(mSensorData[SensorFeedback::SENSOR_FRONT_RIGHT_CENTER] < FRONT_SIDES_AVOIDANCE_DISTANCE-20 && mSensorData[SensorFeedback::SENSOR_FRONT_LEFT_CENTER] < FRONT_SIDES_AVOIDANCE_DISTANCE-20)
		{
			right_speed = scaleSpeed(right_speed, 0.5*FRONT_CENTER_AND_SIDES_AVOIDANCE_IMPACT, FRONT_SIDES_AVOIDANCE_DISTANCE, mSensorData[SensorFeedback::SENSOR_FRONT_CENTER_LEFT], mSensorData[SensorFeedback::SENSOR_FRONT_CENTER_RIGHT]);
		}
		else
		{
			//If there is space front right of the robot, then turn to that side. Otherwise, turn to the other side.
			if(mSensorData[SensorFeedback::SENSOR_FRONT_RIGHT] > FRONT_AVOIDANCE_DISTANCE)
				right_speed = scaleSpeed(right_speed, FRONT_CENTER_AND_SIDES_AVOIDANCE_IMPACT, FRONT_AVOIDANCE_DISTANCE, mSensorData[SensorFeedback::SENSOR_FRONT_CENTER_LEFT], mSensorData[SensorFeedback::SENSOR_FRONT_CENTER_RIGHT]);
			else
				left_speed = scaleSpeed(right_speed, FRONT_CENTER_AND_SIDES_AVOIDANCE_IMPACT, FRONT_AVOIDANCE_DISTANCE, mSensorData[SensorFeedback::SENSOR_FRONT_CENTER_LEFT], mSensorData[SensorFeedback::SENSOR_FRONT_CENTER_RIGHT]);
		}
	}

	//Restore the direction of the speeds when all calculations are done
	left_speed = converted_left < ZERO_SPEED ? -left_speed : left_speed;
	right_speed = converted_right < ZERO_SPEED ? -right_speed : right_speed;

	//Don't go backwards when there is a wall
	if(msg.linear.x < 0 && mSensorData[SensorFeedback::SENSOR_REAR_LEFT] < REAR_HALTING_DISTANCE && mSensorData[SensorFeedback::SENSOR_REAR_RIGHT] < REAR_HALTING_DISTANCE)
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


