/*
 * SafeKeeper.h
 *
 *  Created on: Oct 20, 2011
 *      Author: hans
 */

#ifndef SAFEKEEPER_H_
#define SAFEKEEPER_H_

#include <ros/ros.h>
#include <mobile_base/BaseMotorControl.h>
#include <mobile_base/BumperDisableMotor.h>
#include <mobile_base/DisableMotor.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Float64.h>

#define SAFE_DISTANCE  0.5

enum BumperState
{
	BUMPER_NONE,
	BUMPER_FRONT,
	BUMPER_REAR,
	BUMPER_REAR_LEFT,
	BUMPER_REAR_RIGHT
};

/// Checks collisions and keeps the robot safe by moving away from obstacles
class SafeKeeper
{
private:
	ros::NodeHandle mNodeHandle;    			/// ROS node handle

	ros::Subscriber mUltrasoneDisableMotor_sub;	/// Subcriber to DisableMotor topic from ultrasone sensors
	ros::Subscriber mBumperFeedback_sub;		/// Subcriber to arduino Bumper sensor feedback

	ros::Publisher	mBumperDisableMotor_pub;	/// Publisher for disabling forward/backward movement, triggered by bumper sensors
	ros::Publisher  mMovement_pub;				/// Publisher to control the motors
	BumperState 	mBumperState;

	bool mFrontDisabledByUltrasone;				/// Checks whether motor is disabled by front ultrasone sensors
	bool mRearDisabledByUltrasone;				/// Checks whether motor is disabled by rear ultrasone sensors
public:
	SafeKeeper() :  mBumperState(BUMPER_NONE) {};

	void init();
	void bumperFeedbackCB(const std_msgs::UInt8 &msg);
	void DisabledByUltrasoneCB(const mobile_base::DisableMotor &msg);
};

#endif /* SAFEKEEPER_H_ */
