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
#include <head/Emotion.h>

#define ERROR_STATE_BUMPER_COUNT 6
#define ERROR_STATE_BUMPER_TIME 60.0

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
	ros::Subscriber mSpeed_sub;					/// Subscriber to current speed of mobile base

	ros::Publisher  mMovement_pub;				/// Publisher to control the motors
	ros::Publisher	mEmotion_pub;				///
	BumperState 	mBumperState;				/// Keeps track of the current state for the bumpers
	bool mDisableForward, mDisableBackward;		/// Keeps track of the front and backward buttons of the bumpers

	geometry_msgs::Twist mCurrentSpeed;			/// Keeps track of the current speed of mobile base

	boost::array<int16_t, 10> mSensorData;

	uint32_t mBumperCount;
	double mBumperStartTime;

public:
	SafeKeeper() :  mBumperState(BUMPER_NONE), mBumperCount(0), mBumperStartTime(0.0) {};

	void init();
	void bumperFeedbackCB(const std_msgs::UInt8 &msg);
	void ultrasoneFeedbackCB(const mobile_base::SensorFeedback& msg);
	void speedFeedbackCB(const geometry_msgs::Twist &msg);

	inline ros::NodeHandle* getNodeHandle() { return &mNodeHandle; };
};

#endif /* SAFEKEEPER_H_ */
