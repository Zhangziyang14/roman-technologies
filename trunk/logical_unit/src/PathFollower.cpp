/*
 * PathFollower.cpp
 *
 *  Created on: Dec 20, 2011
 *      Author: hans
 */

#include "logical_unit/PathFollower.h"

double distanceToLine(geometry_msgs::Point p, geometry_msgs::Point a, geometry_msgs::Point b)
{
	double xu = p.x - a.x;
	double yu = p.y - a.y;
	double xv = b.x - a.x;
	double yv = b.y - a.y;
	if (xu * xv + yu * yv < 0)
		return sqrt( (p.x-a.x)*(p.x-a.x) + (p.y-a.y)*(p.y-a.y));

	xu = p.x - b.x;
	yu = p.y - b.y;
	xv = -xv;
	yv = -yv;
	if (xu * xv + yu * yv < 0)
		return sqrt( (p.x-b.x)*(p.x-b.x) + (p.y-b.y)*(p.y-b.y) );

	return fabs( (p.x*(a.y - b.y) + p.y*(b.x - a.x) + (a.x * b.y - b.x * a.y)) / sqrt((b.x - a.x)*(b.x - a.x) + (b.y - a.y)*(b.y - a.y)));
}

PathFollower::PathFollower() : mFollowState(FOLLOW_STATE_IDLE)
{
	std::string pathTopic, speedFeedbackTopic, goalTopic;
	mNodeHandle.param<std::string>("path_topic", pathTopic, "/global_path");
	mNodeHandle.param<std::string>("speed_feedback_topic", speedFeedbackTopic, "/speedFeedbackTopic");
	mNodeHandle.param<std::string>("goal_topic", goalTopic, "/move_base_simple/goal");
	mNodeHandle.param<int>("refresh_rate", mRefreshRate, 5);
	mNodeHandle.param<double>("min_angular_speed", mMinAngularSpeed, 0.1);
	mNodeHandle.param<double>("max_angular_speed", mMaxAngularSpeed, 0.4);
	mNodeHandle.param<double>("min_linear_speed", mMinLinearSpeed, 0.2);
	mNodeHandle.param<double>("max_linear_speed", mMaxLinearSpeed, 0.3);
	mNodeHandle.param<double>("angular_adjustment_speed", mAngularAdjustmentSpeed, 0.05);
	mNodeHandle.param<double>("disable_transition_threshold", mDisableTransitionThreshold, 0.05);
	mNodeHandle.param<double>("final_yaw_tolerance", mFinalYawTolerance, 0.2);
	mNodeHandle.param<double>("yaw_tolerance", mYawTolerance, 0.25 * M_PI);
	mNodeHandle.param<double>("distance_tolerance", mDistanceTolerance, 0.2);
	mNodeHandle.param<double>("reset_distance_tolerance", mResetDistanceTolerance, 0.5);

	mPathSub = mNodeHandle.subscribe(pathTopic, 1, &PathFollower::pathCb, this);
	mCommandPub = mNodeHandle.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
	mGoalPub = mNodeHandle.advertise<geometry_msgs::PoseStamped>(goalTopic, 1);
}

double PathFollower::calculateDiffYaw()
{
	if (getPathSize() == 0)
	{
		//ROS_INFO("No change yaw.");
		return 0.0; // no change needed
	}
	if (getPathSize() == 1)
	{
		//ROS_INFO("Final point yaw.");
		double yaw = tf::getYaw(mPath.front().orientation);
		btVector3 orientation = btVector3(cos(yaw), sin(yaw), 0.0).rotate(btVector3(0,0,1), -tf::getYaw(mRobotPosition.getRotation()));
		return atan2(orientation.getY(), orientation.getX());
	}

	//ROS_INFO("Target yaw.");

	double dx = getNextPoint().x - mRobotPosition.getOrigin().getX();
	double dy = getNextPoint().y - mRobotPosition.getOrigin().getY();
	btVector3 diff = btVector3(dx, dy, 0).rotate(btVector3(0,0,1), -tf::getYaw(mRobotPosition.getRotation()));
	return atan2(diff.getY(), diff.getX());
}

void PathFollower::continuePath()
{
	mOrigin = getNextPoint();

	mPath.pop_front();
	if (getPathSize() == 0)
	{
		mFollowState = FOLLOW_STATE_IDLE;
		ROS_INFO("Goal reached.");
	}
	else
		mFollowState = FOLLOW_STATE_TURNING;

	ROS_INFO("Waypoint reached.");
	geometry_msgs::Twist msg;
	mCommandPub.publish(msg);
}

void PathFollower::clearPath()
{
	while (getPathSize())
		continuePath();
}

void PathFollower::handlePath(tf::TransformListener *transformListener)
{
	geometry_msgs::Twist command;

	// no path? nothing to handle
	if (getPathSize() == 0)
		return;

	// get current position in the map
	try
	{
		transformListener->lookupTransform("/map", "/base_link", ros::Time(0), mRobotPosition);
	}
	catch (tf::TransformException ex)
	{
		//ROS_ERROR("%s",ex.what());
	}

	double robotYaw = tf::getYaw(mRobotPosition.getRotation());
	double diffYaw = calculateDiffYaw();

	ROS_INFO("Follow state: %d Robot pos: (%lf, %lf, %lf). Target pos: (%lf, %lf, %lf). RobotYaw: %lf. FocusYaw: %lf", mFollowState, mRobotPosition.getOrigin().getX(),
			mRobotPosition.getOrigin().getY(), mRobotPosition.getOrigin().getZ(), getNextPoint().x,
			getNextPoint().y, getNextPoint().z, robotYaw, diffYaw);

	geometry_msgs::Point robotPos;
	robotPos.x = mRobotPosition.getOrigin().x();
	robotPos.y = mRobotPosition.getOrigin().y();
	double distToPath = distanceToLine(robotPos, getOrigin(), getNextPoint());
	if (distToPath > mResetDistanceTolerance)
	{
		ROS_INFO("Too far away from path, re-publishing goal, getting new path.");
		geometry_msgs::PoseStamped goal;
		goal.pose = getGoal();
		goal.header.stamp = ros::Time::now();
		mGoalPub.publish(goal);
		clearPath();
	}
	if (fabs(diffYaw) > mYawTolerance)
	{
		ROS_INFO("Yaw too far away from target, turning back.");
		mFollowState = FOLLOW_STATE_TURNING;
	}

	switch (mFollowState)
	{
	case FOLLOW_STATE_TURNING:
		if (fabs(diffYaw) < mFinalYawTolerance)
		{
			mFollowState = FOLLOW_STATE_FORWARD;
			break;
		}

		// if we're already near our goal point and not rotating for final orientation, continue the path
		if (getPathSize() != 1 && reachedNextPoint())
		{
			continuePath();
			break;
		}

		command.angular.z = diffYaw > 0.0 ? getScaledAngularSpeed(diffYaw) : -getScaledAngularSpeed(diffYaw);
		break;
	case FOLLOW_STATE_FORWARD:
		//ROS_INFO("Moving.");
		if (reachedNextPoint())
		{
			continuePath();
			break;
		}

		command.linear.x = getScaledLinearSpeed();
		command.angular.z = diffYaw > 0.0 ? mAngularAdjustmentSpeed : -mAngularAdjustmentSpeed;
		break;
	default:
		break;
	}

	mCommandPub.publish(command);
}

void PathFollower::spin()
{
	tf::TransformListener transformListener;
	ros::Rate refreshRate(mRefreshRate);

	while (ros::ok())
	{
		handlePath(&transformListener);

		refreshRate.sleep();
		ros::spinOnce();
	}
}

void PathFollower::pathCb(const nav_msgs::Path &path)
{
	ROS_INFO("Received new path.");

	mPath.clear();

	for (size_t i = 0; i < path.poses.size(); i++)
		mPath.push_back(path.poses[i].pose);

	// add last point twice for orientation
	mPath.push_back(path.poses[path.poses.size() - 1].pose);

	mFollowState = FOLLOW_STATE_TURNING;
	// first waypoint is usually the starting position
	if (reachedNextPoint())
		continuePath();
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "PathFollower");

	PathFollower pf;

	pf.spin();
	return 0;
}
