/*
 * AutonomeMobileController.h
 *
 *  Created on: 2012-01-20
 *      Author: wilson
 */

#ifndef AUTONOMEMOBILECONTROLLER_H_
#define AUTONOMEMOBILECONTROLLER_H_


#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#include <mobile_base/MotorHandler.h>
#include <mobile_base/PathHandler.h>
#include <nero_msgs/MotorPosition.h>

/// Controller for teleoperation
class AutonomeMobileController
{
protected:
	ros::NodeHandle mNodeHandle;	/// ROS node handle
	ros::Publisher mPositionPub;  	/// Twist message publisher, publishes movement data for motors

	ros::Subscriber mPositionSub;	/// Listens to position commands
	ros::Subscriber mTurnSub;		/// Listens to turning commands

	int mRefreshRate;				/// Rate at which the path gets updated
	PathHandler mPathHandler;		/// Handles received paths

public:
	/// Constructor
	AutonomeMobileController(): mNodeHandle(""), mPathHandler(&mNodeHandle)
	{
		mNodeHandle.param<int>("refresh_rate", mRefreshRate, 5);
	};

	/// Destructor
	~AutonomeMobileController()
	{
		mNodeHandle.shutdown();
	};

	void init();
	void positionCB(const std_msgs::Float32& msg);
	void turnCB(const std_msgs::Float32& msg);
	void scaleTwist(const geometry_msgs::Twist& msg);
	void spin();
};

#endif /* AUTONOMEMOBILECONTROLLER_H_ */
