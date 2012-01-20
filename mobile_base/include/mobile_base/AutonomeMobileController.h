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
#include <mobile_base/MotorHandler.h>
#include <mobile_base/PathFollower.h>

/// Controller for teleoperation
class AutonomeMobileController
{
protected:
	ros::NodeHandle mNodeHandle;	/// ROS node handle
	ros::Publisher mSpeedPub;  		/// Twist message publisher, publishes movement data for motors
	int mRefreshRate;				/// Rate at which the path gets updated
	PathFollower mPathFollower;		/// Handles received paths

public:
	/// Constructor
	AutonomeMobileController(): mNodeHandle("")
	{
		mNodeHandle.param<int>("refresh_rate", mRefreshRate, 5);
	};

	/// Destructor
	~AutonomeMobileController()
	{
		mNodeHandle.shutdown();
	};

	void scaleTwist(const geometry_msgs::Twist& msg);
	void spin();
};

#endif /* AUTONOMEMOBILECONTROLLER_H_ */
