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

enum FollowState
{
	FOLLOW_STATE_IDLE,
	FOLLOW_STATE_TURNING,
	FOLLOW_STATE_FORWARD,
	FOLLOW_STATE_MAX,
};

class PathFollower
{
private:
	ros::NodeHandle mNodeHandle;
	ros::Subscriber mPathSub;
	ros::Subscriber mSpeedFeedbackSub;
	ros::Publisher mCommandPub;
	nav_msgs::Path mPath;
	int mPathIndex;
	tf::StampedTransform mRobotPosition;

	int mRefreshRate;
	double mYawTolerance;
	double mAngularSpeed;
	double mLinearSpeed;

	FollowState mFollowState;
	FollowState mAllowState;
	double mDisableTransitionThreshold;
	double mFinalYawTolerance;

public:
	PathFollower();

	void spin();
	void pathCb(const nav_msgs::Path &path);
	void speedCb(const geometry_msgs::Twist &twist);
	void continuePath();
	void handlePath(tf::TransformListener *transformListener);
};

#endif /* PATHFOLLOWER_H_ */
