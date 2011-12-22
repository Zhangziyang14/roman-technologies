/*
 * PathFollower.cpp
 *
 *  Created on: Dec 20, 2011
 *      Author: hans
 */

#include "logical_unit/PathFollower.h"

PathFollower::PathFollower() : mPathIndex(0), mFollowState(FOLLOW_STATE_IDLE), mAllowState(FOLLOW_STATE_IDLE)
{
	std::string pathTopic, speedFeedbackTopic;
	mNodeHandle.param<std::string>("path_topic", pathTopic, "/global_path");
	mNodeHandle.param<std::string>("speed_feedback_topic", pathTopic, "/speedFeedbackTopic");
	mNodeHandle.param<int>("refresh_rate", mRefreshRate, 5);
	mNodeHandle.param<double>("yaw_tolerance", mYawTolerance, 0.2);
	mNodeHandle.param<double>("angular_speed", mAngularSpeed, 0.2);
	mNodeHandle.param<double>("linear_speed", mLinearSpeed, 0.2);
	mNodeHandle.param<double>("disable_transition_threshold", mDisableTransitionThreshold, 0.05);
	mNodeHandle.param<double>("final_yaw_tolerance", mFinalYawTolerance, 0.1);

	mPathSub = mNodeHandle.subscribe(pathTopic, 1, &PathFollower::pathCb, this);
	mSpeedFeedbackSub = mNodeHandle.subscribe(speedFeedbackTopic, 1, &PathFollower::pathCb, this);
	mCommandPub = mNodeHandle.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
}

void PathFollower::continuePath()
{
	mPathIndex++;
	if (int(mPath.poses.size()) == mPathIndex + 1)
	{
		ROS_INFO("Goal reached.");
		mPath.poses.clear();
		mPathIndex = 0;

		geometry_msgs::Twist msg;
		mCommandPub.publish(msg);
	}
}

void PathFollower::handlePath(tf::TransformListener *transformListener)
{
	geometry_msgs::Twist command;

	// no path? nothing to handle
	if (mPath.poses.size() == 0)
		return;

	// get current position in the map
	transformListener->lookupTransform("/map", "/base_link", ros::Time(0), mRobotPosition);

	ROS_INFO("Robot pos: (%lf, %lf, %lf). Target pos: (%lf, %lf, %lf).", mRobotPosition.getOrigin().getX(),
			mRobotPosition.getOrigin().getY(), mRobotPosition.getOrigin().getZ(), mPath.poses[mPathIndex+1].pose.position.x,
			mPath.poses[mPathIndex+1].pose.position.y, mPath.poses[mPathIndex+1].pose.position.z);

	double robotYaw = tf::getYaw(mRobotPosition.getRotation());
	double targetYaw = tf::getYaw(mPath.poses[mPathIndex+1].pose.orientation);

	btVector3 robotPos(mPath.poses[mPathIndex+1].pose.position.x, mPath.poses[mPathIndex+1].pose.position.y, mPath.poses[mPathIndex+1].pose.position.z);
	double dist = mRobotPosition.getOrigin().distance(robotPos);
	if (dist < 0.2 && (mPathIndex + 1 != int(mPath.poses.size() - 1) || fabs(robotYaw - targetYaw) < mFinalYawTolerance))
	{
		ROS_INFO("Reached waypoint.");
		continuePath();
		return;
	}

	double requiredYaw;
	if (mPathIndex + 1 == int(mPath.poses.size() - 1))
	{
		double dx = mPath.poses[mPathIndex+1].pose.position.x - mRobotPosition.getOrigin().getX();
		double dy = mPath.poses[mPathIndex+1].pose.position.y - mRobotPosition.getOrigin().getY();
		if (dx == 0.0)
			dx = 0.01;
		requiredYaw = atan(fabs(dy / dx));
		if (dx < 0.0)
		{
			if (dy < 0.0)
				requiredYaw = -M_PI + requiredYaw;
			else
				requiredYaw = M_PI - requiredYaw;
		}
		else if (dy < 0.0)
			requiredYaw = -1 * requiredYaw;
	}
	else
	{
		requiredYaw = targetYaw;
		ROS_INFO("Turning for final orientation.");
	}
	double diff = requiredYaw - robotYaw;

	// do we need to go rotate?
	if ((mAllowState == FOLLOW_STATE_IDLE || mAllowState == FOLLOW_STATE_TURNING) &&
			fabs(diff) > mYawTolerance)
	{
		ROS_INFO("Trying to rotate; yaw: %lf, requiredYaw: %lf, diff: %lf", robotYaw, requiredYaw, diff);

		mFollowState = FOLLOW_STATE_TURNING;
		command.angular.z = mAngularSpeed;
		if (diff < 0.0)
			command.angular.z = -command.angular.z;
	}
	else if (mAllowState == FOLLOW_STATE_IDLE || mAllowState == FOLLOW_STATE_FORWARD)// we need to go forward
		command.linear.x = mLinearSpeed;

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
	mPath = path;
	mPathIndex = 0;
}

void PathFollower::speedCb(const geometry_msgs::Twist &twist)
{
	double vel_left = twist.linear.x + twist.angular.z / 2;
	double vel_right = twist.linear.x - twist.angular.z / 2;

	if ((fabs(vel_left) > mDisableTransitionThreshold || fabs(vel_right) > mDisableTransitionThreshold) &&
			mFollowState != FOLLOW_STATE_IDLE)
		mAllowState = mFollowState;
	else
		mAllowState = FOLLOW_STATE_IDLE;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "PathFollower");

	PathFollower pf;

	pf.spin();
	return 0;
}
