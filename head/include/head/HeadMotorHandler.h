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

class HeadMotorHandler
{

protected:
	ros::NodeHandle mNodeHandle;			/// ROS node handle

	ros::Subscriber mPositionSub;			/// Listens to pose messages for positioning of the head
	ros::Publisher mPositionPub;			/// Publishes current head position
	geometry_msgs::Pose mCurrentPose;		/// Keeps track of the current pose of the head

	DynamixelMotor mDynamixel;				/// Dynamixel motor

	MotorId mMotorId;						/// Id of the motor that is currently being controlled

public:
	/// Constructor
	HeadMotorHandler() : mNodeHandle("~"), mDynamixel(DYNAMIXEL_ID) { }

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
