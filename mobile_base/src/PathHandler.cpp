/*
 * PathHandler.cpp
 *
 *  Created on: Jan 20, 2012
 *      Author: hans
 */

#include "mobile_base/PathHandler.h"

/**
 * Calculates a point from point p to the line described by points a and b.
 */
geometry_msgs::Point closestPointOnLine(geometry_msgs::Point p, geometry_msgs::Point a, geometry_msgs::Point b)
{
	geometry_msgs::Point result;
	geometry_msgs::Point ap;
	ap.x = p.x - a.x;
	ap.y = p.y - a.y;
	geometry_msgs::Point ab;
	ab.x = b.x - a.x;
	ab.y = b.y - a.y;
	float ab2 = ab.x*ab.x + ab.y*ab.y;
	float ap_ab = ap.x*ab.x + ap.y*ab.y;
	float t = ap_ab / ab2;

	if (t < 0.0f)
		t = 0.0f;
	else if (t > 1.0f)
		t = 1.0f;

	result.x = a.x + ab.x * t;
	result.y = a.y + ab.y * t;
	return result;
}

double distanceBetweenPoints(geometry_msgs::Point a, geometry_msgs::Point b)
{
	double dx = a.x - b.x;
	double dy = a.y - b.y;
	double dz = a.z - b.z;
	return sqrt(dx*dx + dy*dy + dz*dz);
}

/**
 * Constructor
 */
PathHandler::PathHandler(ros::NodeHandle *nodeHandle) : mCurrentPathIndex(0), mFollowState(FOLLOW_STATE_IDLE)
{
	std::string localPathTopic, pathTopic, goalTopic, followStateTopic;
	nodeHandle->param<std::string>("path_topic", pathTopic, "/global_path");
	nodeHandle->param<std::string>("local_path_topic", localPathTopic, "/local_path");
	nodeHandle->param<std::string>("goal_topic", goalTopic, "/move_base_simple/goal");
	nodeHandle->param<std::string>("follow_state_topic", followStateTopic, "/follow_state");
	nodeHandle->param<double>("min_angular_speed", mMinAngularSpeed, 0.2);
	nodeHandle->param<double>("max_angular_speed", mMaxAngularSpeed, 0.3);
	nodeHandle->param<double>("min_linear_speed", mMinLinearSpeed, 0.2);
	nodeHandle->param<double>("max_linear_speed", mMaxLinearSpeed, 0.4);
	nodeHandle->param<double>("final_yaw_tolerance", mFinalYawTolerance, 0.1);
	nodeHandle->param<double>("reset_distance_tolerance", mResetDistanceTolerance, 0.5);
	nodeHandle->param<double>("distance_tolerance", mDistanceTolerance, 0.05);
	nodeHandle->param<double>("look_forward_distance", mLookForwardDistance, 0.5);
	nodeHandle->param<double>("max_angle_linear_scale", mMaxAngleLinearScale, M_PI / 6.0);

	mPathSub = nodeHandle->subscribe(pathTopic, 1, &PathHandler::pathCb, this);
	mCommandPub = nodeHandle->advertise<geometry_msgs::Twist>("/cmd_vel", 1);
	mGoalPub = nodeHandle->advertise<geometry_msgs::PoseStamped>(goalTopic, 1);
	mFollowStatePub = nodeHandle->advertise<std_msgs::UInt8>(followStateTopic, 1);
	mLocalPathPub = nodeHandle->advertise<nav_msgs::Path>(localPathTopic, 1);
}

double PathHandler::angleTo(geometry_msgs::Point p)
{
	double dx = p.x - mRobotPosition.getOrigin().getX();
	double dy = p.y - mRobotPosition.getOrigin().getY();
	btVector3 diff = btVector3(dx, dy, 0).rotate(btVector3(0,0,1), -tf::getYaw(mRobotPosition.getRotation()));
	return atan2(diff.getY(), diff.getX());
}

geometry_msgs::Point PathHandler::getPointOnPathWithDist(geometry_msgs::Point p, double dist)
{
	if (getPathSize() <= 1) // nothing to do here
	{
		ROS_ERROR("Called getPointOnPathWithDist with a too small path!");
		return p;
	}

	geometry_msgs::Point result;
	double d = 0.0;
	uint32_t i = 0;
	for (i = mCurrentPathIndex; i < getPathSize() - 1; i++)
	{
		d = distanceBetweenPoints(p, mPath[i+1].position);
		if (dist >= d)
			dist = dist - d;
		else
			break;
		p = mPath[i+1].position;
	}

	if (i < getPathSize() - 1)
	{
		double scale = dist / distanceBetweenPoints(p, mPath[i+1].position);
		result.x = p.x + scale * (mPath[i+1].position.x - p.x);
		result.y = p.y + scale * (mPath[i+1].position.y - p.y);
	}
	else
		result = mPath[getPathSize() - 1].position;
	return result;
}

/**
 * Publishes the total distance of the remaining path
 */
double PathHandler::getPathLength()
{
	geometry_msgs::Point p;
	p.x = mRobotPosition.getOrigin().getX();
	p.y = mRobotPosition.getOrigin().getY();
	p.z = mRobotPosition.getOrigin().getZ();
	double length = 0.f;
	for (uint32_t i = mCurrentPathIndex + 1; i < mPath.size(); i++)
	{
		length += distanceBetweenPoints(p, mPath[i].position);
		p = mPath[i].position;
	}
	return length;
}

/**
 * Publishes the robot state
 */
void PathHandler::publishState()
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

void PathHandler::publishLocalPath(double amplitude, double yaw)
{
	nav_msgs::Path path;
	path.header.frame_id = "/base_link";

	geometry_msgs::PoseStamped p;
	p.header.frame_id = "/base_link";
	p.header.stamp = ros::Time::now();
	p.pose.position.x = 0.0;
	p.pose.position.y = 0.0;
	p.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);

	path.poses.push_back(p);

	p.pose.position.x = std::max(0.5, amplitude) * cos(yaw);
	p.pose.position.y = std::max(0.5, amplitude) * sin(yaw);
	path.poses.push_back(p);

	mLocalPathPub.publish(path);
}

/**
 * Clears path (continously calls continuePath until empty)
 */
void PathHandler::clearPath()
{
	mPath.clear();
	mCurrentPathIndex = 0;
}

/**
 * Updates path logic.
 */
void PathHandler::updatePath()
{
	geometry_msgs::Twist command;

	publishState();

	// no path? nothing to handle
	if (getPathSize() == 0)
		return;

	// update our position if possible.
	if (updateCurrentPosition() == false)
	{
		ROS_ERROR("Failed to update robot position.");
		return;
	}

	if (mCurrentPathIndex < getPathSize() - 1) // we are moving along a line segment in the path
	{
		geometry_msgs::Point robotPos;
		robotPos.x = mRobotPosition.getOrigin().x();
		robotPos.y = mRobotPosition.getOrigin().y();
		geometry_msgs::Point closestOnPath = closestPointOnLine(robotPos, mPath[mCurrentPathIndex].position, mPath[mCurrentPathIndex + 1].position);

		ROS_INFO("robotPos[%lf, %lf], closestOnPath[%lf, %lf]", robotPos.x, robotPos.y, closestOnPath.x, closestOnPath.y);

		if (distanceBetweenPoints(closestOnPath, robotPos) > mResetDistanceTolerance)
		{
			ROS_INFO("Drove too far away from path, re-sending goal.");
			mFollowState = FOLLOW_STATE_IDLE;
			resendGoal();
			clearPath();
			return;
		}

		geometry_msgs::Point pointOnPath = getPointOnPathWithDist(closestOnPath, mLookForwardDistance);

		ROS_INFO("closestOnPath[%lf, %lf], pointOnPath[%lf, %lf]", closestOnPath.x, closestOnPath.y, pointOnPath.x, pointOnPath.y);

		double angle = angleTo(pointOnPath);

		double robotYaw = tf::getYaw(mRobotPosition.getRotation()); // only used for printing
		ROS_INFO("Follow state: %d Robot pos: (%lf, %lf, %lf). Target pos: (%lf, %lf, %lf). RobotYaw: %lf. FocusYaw: %lf", mFollowState, mRobotPosition.getOrigin().getX(),
				mRobotPosition.getOrigin().getY(), mRobotPosition.getOrigin().getZ(), pointOnPath.x,
				pointOnPath.y, pointOnPath.z, robotYaw, angle);

		ROS_INFO("Angle to path: %f", angle);

		command.linear.x = getScaledLinearSpeed(angle);
		command.angular.z = getScaledAngularSpeed(angle);

		if (distanceBetweenPoints(closestOnPath, mPath[mCurrentPathIndex + 1].position) < mDistanceTolerance)
		{
			ROS_INFO("Moving to next line segment on path.");
			mCurrentPathIndex++;
		}

		publishLocalPath(command.linear.x * 3, angle);
	}
	else // only rotate for final yaw
	{
		double yaw = tf::getYaw(mPath[getPathSize() - 1].orientation);
		btVector3 orientation = btVector3(cos(yaw), sin(yaw), 0.0).rotate(btVector3(0,0,1), -tf::getYaw(mRobotPosition.getRotation()));
		double angle = atan2(orientation.getY(), orientation.getX());

		ROS_INFO("Angle to final orientation: %f", angle);

		if (fabs(angle) > mFinalYawTolerance)
			command.angular.z = getScaledAngularSpeed(angle, true);
		else
		{
			// path is done!
			mFollowState = FOLLOW_STATE_FINISHED;
			clearPath();
		}

		publishLocalPath(1.0, angle);
	}

	mCommandPub.publish(command);
}

/**
 * Updates current position of the robot.
 */
bool PathHandler::updateCurrentPosition()
{
	// get current position in the map
	try
	{
		mTransformListener.lookupTransform("/map", "/base_link", ros::Time(0), mRobotPosition);
	}
	catch (tf::TransformException &ex)
	{
		//ROS_ERROR("%s",ex.what());
		return false;
	}

	return true;
}

/**
 * Receives path from PathPlanner and starts moving towards goal.
 */
void PathHandler::pathCb(const nav_msgs::Path &path)
{
	ROS_INFO("Received new path.");

	if (path.poses.size() == 0)
	{
		ROS_ERROR("Received empty path.");
		return;
	}

	clearPath();
	mCurrentPathIndex = 0;

	for (size_t i = 0; i < path.poses.size(); i++)
		mPath.push_back(path.poses[i].pose);

	mFollowState = FOLLOW_STATE_BUSY;
}
