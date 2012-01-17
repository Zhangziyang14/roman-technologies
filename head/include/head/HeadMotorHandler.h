/*
 * HeadMotorHandler.h
 *
 *  Created on: 2012-01-13
 *      Author: kelvin
 */

#ifndef HEADMOTORHANDLER_H_
#define HEADMOTORHANDLER_H_

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <motors/DynamixelMotor.h>

#define TILT_UPPER_LIMIT	1.25
#define TILT_LOWER_LIMIT	(-1)

#define PAN_UPPER_LIMIT		1.25
#define PAN_LOWER_LIMIT		(-1)

class HeadMotorHandler
{

protected:
	ros::NodeHandle mNodeHandle;			/// ROS node handle

	ros::Subscriber mPositionSub;			/// Listens to pose messages for positioning of the head
	ros::Publisher mPositionPub;			/// Publishes current head position
	geometry_msgs::Pose mCurrentPose;		/// Keeps track of the current pose of the head

	DynamixelMotor mTilt;					/// Dynamixel motor for tilting the head
	DynamixelMotor mPan;					/// Dynamixel motor for panning the head

	MotorId mMotorId;						/// Id of the motor that is currently being controlled

public:
	/// Constructor
	HeadMotorHandler() : mNodeHandle("~"), mTilt(TILT_ID), mPan(PAN_ID) { }

	/// Destructor
	/** Delete motor interface, close serial port, and shut down node handle */
	~HeadMotorHandler()
	{
		mNodeHandle.shutdown();
	}

	/// Initialise node
	void init(char *path);

	void positionCB(const geometry_msgs::Pose& msg);
	void publishHeadPosition();
};


#endif /* HEADMOTORHANDLER_H_ */
