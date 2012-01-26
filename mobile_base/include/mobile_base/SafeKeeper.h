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
#include <mobile_base/SensorFeedback.h>
#include <mobile_base/position.h>
#include <std_msgs/UInt8.h>


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

	ros::Subscriber mBumperFeedback_sub;		/// Subscriber to Arduino bumper sensor feedback
	ros::Subscriber mUltrasone_sub;				/// Subscriber to Arduino ultrasone sensors feedback

	ros::Publisher  mMovement_pub;				/// Publisher to control the motors
	BumperState 	mBumperState;				/// Keeps track of the current state for the bumpers
	bool mDisableForward, mDisableBackward;		/// Keeps track of the front and backward buttons of the bumpers

	boost::array<int16_t, 10> mSensorData;

public:
	SafeKeeper() :  mBumperState(BUMPER_NONE) {};

	void init();
	void bumperFeedbackCB(const std_msgs::UInt8 &msg);
	void ultrasoneFeedbackCB(const mobile_base::SensorFeedback& msg);

	inline ros::NodeHandle* getNodeHandle() { return &mNodeHandle; };
};

#endif /* SAFEKEEPER_H_ */
