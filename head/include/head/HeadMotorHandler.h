/*
 * HeadMotorHandler.h
 *
 *  Created on: 2012-01-13
 *      Author: kelvin
 */

#ifndef HEADMOTORHANDLER_H_
#define HEADMOTORHANDLER_H_

#include <ros/ros.h>
#include <head/PitchYaw.h>
#include <motors/DynamixelMotor.h>

#define PITCH_UPPER_LIMIT	1.1
#define PITCH_LOWER_LIMIT	(-0.6)

#define YAW_UPPER_LIMIT		(0.35)
#define YAW_LOWER_LIMIT		(-0.35)

#define YAW_OFFSET			(-1)
#define PITCH_OFFSET		(-0.5)

class HeadMotorHandler
{

protected:
	ros::NodeHandle mNodeHandle;			/// ROS node handle

	ros::Subscriber mPositionSub;			/// Listens to pose messages for positioning of the head
	ros::Publisher mPositionPub;			/// Publishes current head position
	head::PitchYaw mCurrentPose;			/// Keeps track of the current pose of the head

	DynamixelMotor mPitch;					/// Dynamixel motor for tilting the head
	DynamixelMotor mYaw;					/// Dynamixel motor for panning the head

	MotorId mMotorId;						/// Id of the motor that is currently being controlled

public:
	/// Constructor
	HeadMotorHandler() : mNodeHandle("~"), mPitch(TILT_ID), mYaw(PAN_ID) { }

	/// Destructor
	/** Delete motor interface, close serial port, and shut down node handle */
	~HeadMotorHandler()
	{
		mNodeHandle.shutdown();
	}

	/// Initialise node
	void init(char *path);

	void positionCB(const head::PitchYaw &msg);
	void publishHeadPosition();
};


#endif /* HEADMOTORHANDLER_H_ */
