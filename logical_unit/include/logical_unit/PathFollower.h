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
	ros::Publisher mCommandPub;
	ros::Publisher mGoalPub;
	std::list<geometry_msgs::Pose> mPath;
	geometry_msgs::Point mFocusPoint;
	geometry_msgs::Point mOrigin;
	geometry_msgs::Quaternion mFocusOrientation;
	tf::StampedTransform mRobotPosition;

	int mRefreshRate;
	double mMinAngularSpeed, mMaxAngularSpeed;
	double mAngularAdjustmentSpeed;
	double mMinLinearSpeed, mMaxLinearSpeed;

	FollowState mFollowState;
	double mDisableTransitionThreshold;
	double mYawTolerance;
	double mFinalYawTolerance;
	double mDistanceTolerance;
	double mResetDistanceTolerance;

	inline geometry_msgs::Pose getGoal() { return mPath.back(); };
	inline geometry_msgs::Point getNextPoint() { return mPath.front().position; };
	inline geometry_msgs::Point getOrigin() { return mOrigin; };
	inline int getPathSize() { return mPath.size(); };
	inline bool reachedNextPoint()
	{
		btVector3 np(getNextPoint().x, getNextPoint().y, getNextPoint().z);
		return np.distance(mRobotPosition.getOrigin()) < mDistanceTolerance;
	};
	inline double getScaledLinearSpeed()
	{
		btVector3 np(getNextPoint().x, getNextPoint().y, getNextPoint().z);
		double scale = (std::max(std::min(np.distance(mRobotPosition.getOrigin()), 3.0), 1.0) - 1.0) / 2.0;
		return scale * (mMaxLinearSpeed - mMinLinearSpeed) + mMinLinearSpeed;
	};
	inline double getScaledAngularSpeed(double rotationAngle)
	{
		double scale = fabs(rotationAngle) / M_PI;
		return scale * (mMaxAngularSpeed - mMinAngularSpeed) + mMinAngularSpeed;
	};

	double calculateDiffYaw();
public:
	PathFollower();

	void spin();
	void pathCb(const nav_msgs::Path &path);
	void continuePath();
	void clearPath();
	void handlePath(tf::TransformListener *transformListener);
};

#endif /* PATHFOLLOWER_H_ */
