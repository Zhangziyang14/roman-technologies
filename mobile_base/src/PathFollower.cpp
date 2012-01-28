/*
 * PathFollower.cpp
 *
 *  Created on: Jan 20, 2012
 *      Author: hans
 */

#include "mobile_base/PathFollower.h"

/**
 * Calculates a distance from point p to the line described by points a and b.
 */
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

	return fabs((p.x*(a.y - b.y) + p.y*(b.x - a.x) + (a.x * b.y - b.x * a.y)) / sqrt((b.x - a.x)*(b.x - a.x) + (b.y - a.y)*(b.y - a.y)));
}

float distanceBetweenPoints(geometry_msgs::Point a, geometry_msgs::Point b)
{
	float dx = a.x - b.x;
	float dy = a.y - b.y;
	float dz = a.z - b.z;
	return sqrtf(dx*dx + dy*dy + dz*dz);
}

/**
 * Constructor
 */
PathFollower::PathFollower(ros::NodeHandle *nodeHandle) : mLocalPlanner(nodeHandle), mFollowState(FOLLOW_STATE_IDLE)
{
	std::string pathTopic, speedFeedbackTopic, goalTopic, pathLengthTopic, followStateTopic;
	nodeHandle->param<std::string>("path_topic", pathTopic, "/global_path");
	nodeHandle->param<std::string>("speed_feedback_topic", speedFeedbackTopic, "/speedFeedbackTopic");
	nodeHandle->param<std::string>("goal_topic", goalTopic, "/move_base_simple/goal");
	nodeHandle->param<std::string>("path_length_topic", pathLengthTopic, "/path_length");
	nodeHandle->param<std::string>("follow_state_topic", followStateTopic, "/follow_state");
	nodeHandle->param<double>("min_angular_speed", mMinAngularSpeed, 0.1);
	nodeHandle->param<double>("max_angular_speed", mMaxAngularSpeed, 0.4);
	nodeHandle->param<double>("min_linear_speed", mMinLinearSpeed, 0.1);
	nodeHandle->param<double>("max_linear_speed", mMaxLinearSpeed, 0.3);
	nodeHandle->param<double>("angular_adjustment_speed", mAngularAdjustmentSpeed, 0.05);
	nodeHandle->param<double>("final_yaw_tolerance", mFinalYawTolerance, 0.2);
	nodeHandle->param<double>("yaw_tolerance", mYawTolerance, 0.25 * M_PI);
	nodeHandle->param<double>("distance_tolerance", mDistanceTolerance, 0.2);
	nodeHandle->param<double>("reset_distance_tolerance", mResetDistanceTolerance, 0.5);

	mPathSub = nodeHandle->subscribe(pathTopic, 1, &PathFollower::pathCb, this);
	mCommandPub = nodeHandle->advertise<geometry_msgs::Twist>("/cmd_vel", 1);
	mGoalPub = nodeHandle->advertise<geometry_msgs::PoseStamped>(goalTopic, 1);
	mPathLengthPub = nodeHandle->advertise<std_msgs::Float32>(pathLengthTopic, 1);
	mFollowStatePub = nodeHandle->advertise<std_msgs::UInt8>(followStateTopic, 1);
}


/**
 * Returns the yaw that needs to be turned.
 */
double PathFollower::calculateDiffYaw()
{
	// no path, no yaw
	if (getPathSize() == 0)
	{
		//ROS_INFO("No change yaw.");
		return 0.0; // no change needed
	}
	// last point is only for final orientation
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

/**
 * Continue to the next waypoint.
 */
void PathFollower::continuePath()
{
	mOrigin = getNextPoint();

	mPath.pop_front();
	if (getPathSize() == 0)
	{
		mFollowState = FOLLOW_STATE_FINISHED;
		ROS_INFO("Goal reached.");
	}
	else
		mFollowState = FOLLOW_STATE_TURNING;

	ROS_INFO("Waypoint reached.");
	geometry_msgs::Twist msg;
	mCommandPub.publish(msg);
}

/**
 * Publishes the total distance of the remaining path
 */
void PathFollower::publishPathLength()
{
	if (mPathLengthPub.getNumSubscribers() == 0)
		return;

	geometry_msgs::Point a, b;
	a.x = mRobotPosition.getOrigin().getX();
	a.y = mRobotPosition.getOrigin().getY();
	a.z = mRobotPosition.getOrigin().getZ();
	std_msgs::Float32 msg;
	for (std::list<geometry_msgs::Pose>::iterator i = mPath.begin(); i != mPath.end(); i++)
	{
		msg.data += distanceBetweenPoints(a, i->position);
	}
	mPathLengthPub.publish(msg);
}

/**
 * Publishes the robot state
 */
void PathFollower::publishState()
{
	if (mFollowStatePub.getNumSubscribers())
	{
		std_msgs::UInt8 msg;
		msg.data = mFollowState;
		mFollowStatePub.publish(msg);
	}

	// only send this state once
	if (mFollowState == FOLLOW_STATE_FINISHED)
		mFollowState = FOLLOW_STATE_IDLE;
}

/**
 * Clears path (continously calls continuePath until empty)
 */
void PathFollower::clearPath()
{
	while (getPathSize())
		continuePath();
}

/**
 * Updates path logic.
 */
void PathFollower::updatePath()
{
	geometry_msgs::Twist command;

	publishState();

	// no path? nothing to handle
	if (getPathSize() == 0)
		return;

	// update our position if possible.
	if (updateCurrentPosition() == false)
		return;

	double robotYaw = tf::getYaw(mRobotPosition.getRotation()); // only used for printing
	double diffYaw = calculateDiffYaw();

	ROS_INFO("Follow state: %d Robot pos: (%lf, %lf, %lf). Target pos: (%lf, %lf, %lf). RobotYaw: %lf. FocusYaw: %lf", mFollowState, mRobotPosition.getOrigin().getX(),
			mRobotPosition.getOrigin().getY(), mRobotPosition.getOrigin().getZ(), getNextPoint().x,
			getNextPoint().y, getNextPoint().z, robotYaw, diffYaw);

	geometry_msgs::Point robotPos;
	robotPos.x = mRobotPosition.getOrigin().x();
	robotPos.y = mRobotPosition.getOrigin().y();

	// check if we are too far away from our path. If so, reset our path.
	double distToPath = distanceToLine(robotPos, getOrigin(), getNextPoint());
	if (distToPath > mResetDistanceTolerance)
	{
		ROS_INFO("Too far away from path, re-publishing goal, getting new path.");
		geometry_msgs::PoseStamped goal;
		goal.pose = getGoal();
		goal.header.stamp = ros::Time::now();
		mGoalPub.publish(goal);
		clearPath();
		return;
	}

	// Did we turn away too much? Set state back to turning so we can correct it.
	if (mFollowState != FOLLOW_STATE_TURNING && fabs(diffYaw) > mYawTolerance)
	{
		ROS_INFO("Yaw too far away from target, turning back.");
		mFollowState = FOLLOW_STATE_TURNING;
	}

	switch (mFollowState)
	{
	case FOLLOW_STATE_TURNING:
		// Are we done turning, then change state to move forward
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

		if (getPathSize() == 1)
		{
			continuePath();
			break;
		}

		// Did we reach our waypoint? Then continue to the next waypoint.
		if (reachedNextPoint())
		{
			continuePath();
			break;
		}

		command.linear.x = getScaledLinearSpeed();
		// minor angular adjustments during forward movement.
		command.angular.z = diffYaw > 0.0 ? mAngularAdjustmentSpeed : -mAngularAdjustmentSpeed;
		mLocalPlanner.scaleTwist(command);
		break;
	default:
		break;
	}

	publishPathLength();
	mCommandPub.publish(command);
}

/**
 * Updates current position of the robot.
 */
bool PathFollower::updateCurrentPosition()
{
	// get current position in the map
	try
	{
		mTransformListener.lookupTransform("/map", "/base_link", ros::Time(0), mRobotPosition);
	}
	catch (tf::TransformException ex)
	{
		//ROS_ERROR("%s",ex.what());
		return false;
	}

	return true;
}

/**
 * Receives path from PathPlanner and starts moving towards goal.
 */
void PathFollower::pathCb(const nav_msgs::Path &path)
{
	ROS_INFO("Received new path.");

	if (path.poses.size() == 0)
	{
		ROS_ERROR("Received empty path.");
		return;
	}

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
