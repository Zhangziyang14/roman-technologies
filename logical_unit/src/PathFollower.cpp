/*
 * PathFollower.cpp
 *
 *  Created on: Dec 20, 2011
 *      Author: hans
 */

#include "logical_unit/PathFollower.h"

PathFollower::PathFollower() : mPathIndex(0)
{
	std::string pathTopic;
	mNodeHandle.param<std::string>("path_topic", pathTopic, "/global_path");
	mNodeHandle.param<int>("refresh_rate", mRefreshRate, 5);
	mNodeHandle.param<double>("yaw_tolerance", mYawTolerance, 0.2);
	mNodeHandle.param<double>("angular_speed", mAngularSpeed, 0.2);
	mNodeHandle.param<double>("linear_speed", mLinearSpeed, 0.2);

	mPathSub = mNodeHandle.subscribe(pathTopic, 1, &PathFollower::pathCb, this);
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

	btVector3 robotPos(mPath.poses[mPathIndex+1].pose.position.x, mPath.poses[mPathIndex+1].pose.position.y, mPath.poses[mPathIndex+1].pose.position.z);
	double dist = mRobotPosition.getOrigin().distance(robotPos);
	if (dist < 0.2)
	{
		ROS_INFO("Reached waypoint.");
		continuePath();
		return;
	}

	double yaw = tf::getYaw(mRobotPosition.getRotation());

	double dx = mPath.poses[mPathIndex+1].pose.position.x - mRobotPosition.getOrigin().getX();
	double dy = mPath.poses[mPathIndex+1].pose.position.y - mRobotPosition.getOrigin().getY();
	if (dx == 0.0)
		dx = 0.01;
	double requiredYaw = atan(fabs(dy / dx));
	if (dx < 0.0)
	{
		if (dy < 0.0)
			requiredYaw = -M_PI + requiredYaw;
		else
			requiredYaw = M_PI - requiredYaw;
	}
	else if (dy < 0.0)
		requiredYaw = -1 * requiredYaw;
	double diff = requiredYaw - yaw;

	// do we need to go rotate?
	if (fabs(diff) > mYawTolerance)
	{
		ROS_INFO("Trying to rotate; yaw: %lf, requiredYaw: %lf, diff: %lf", yaw, requiredYaw, diff);

		command.angular.z = mAngularSpeed;
		if (diff < 0.0)
			command.angular.z = -command.angular.z;

		mCommandPub.publish(command);
		return;
	}

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

int main(int argc, char **argv)
{
	ros::init(argc, argv, "PathFollower");

	PathFollower pf;

	pf.spin();
	return 0;
}
