/*
 * PathFollower.h
 *
 *  Created on: Dec 20, 2011
 *      Author: hans
 */

#ifndef PATHFOLLOWER_H_
#define PATHFOLLOWER_H_

#include "ros/ros.h"
#include "nav_msgs/Path.h"
#include "tf/transform_listener.h"

class PathFollower
{
private:
	ros::NodeHandle mNodeHandle;
	ros::Subscriber mPathSub;
	ros::Publisher mCommandPub;
	nav_msgs::Path mPath;
	int mPathIndex;
	tf::StampedTransform mRobotPosition;

	int mRefreshRate;
	double mYawTolerance;
	double mAngularSpeed;
	double mLinearSpeed;

public:
	PathFollower();

	void spin();
	void pathCb(const nav_msgs::Path &path);
	void continuePath();
	void handlePath(tf::TransformListener *transformListener);
};

#endif /* PATHFOLLOWER_H_ */
