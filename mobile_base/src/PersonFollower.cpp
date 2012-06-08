/*
 * PersonFollower.cpp
 *
 *  Created on: Jun 5, 2012
 *      Author: hans
 */

#include "mobile_base/PersonFollower.h"

PersonFollower::PersonFollower() :
	mFollowing(false)
{
	mSpeedPub		= mNodeHandle.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
	mPersonPosSub	= mNodeHandle.subscribe("/PersonTracker/point", 1, &PersonFollower::positionCb, this);

	mNodeHandle.param<double>("/PersonFollower/max_linear", mMaxLinearSpeed, 1.0);
	mNodeHandle.param<double>("/PersonFollower/max_angular", mMaxAngularSpeed, 0.5);
	mNodeHandle.param<double>("/PersonFollower/follow_distance", mFollowDistance, 1.0);
	mNodeHandle.param<double>("/PersonFollower/max_speed_distance", mMaxSpeedDistance, 1.0);
	mNodeHandle.param<double>("/PersonFollower/max_speed_angle", mMaxSpeedAngle, 0.5);
}

void PersonFollower::positionCb(const geometry_msgs::PointStampedPtr &point)
{
	if (point->header.frame_id != "/base_link")
	{
		ROS_ERROR("Person position is not in /base_link! It is in %s.", point->header.frame_id.c_str());
		return;
	}

	if (point->point.x && point->point.y && point->point.z)
	{
		mPersonLoc = point->point;
		mFollowing = true;
	}
	else
		sendSpeed(0.0, 0.0);
}

void PersonFollower::sendSpeed(double linear, double angular)
{
	geometry_msgs::Twist msg;
	msg.linear.x = linear;
	msg.angular.z = angular;
	mSpeedPub.publish(msg);
}

void PersonFollower::updateFollow()
{
	if (isnan(mPersonLoc.x) || isnan(mPersonLoc.y))
	{
		ROS_ERROR("Person location is NaN.");
		return;
	}

	// calculate the distance to the person
	double dist = sqrt(mPersonLoc.x*mPersonLoc.x + mPersonLoc.y*mPersonLoc.y);

	if (dist > MAX_DISTANCE)
	{
		ROS_ERROR("PersonFollower distance is larger than the maximum distance!");
		return;
	}

	ROS_INFO("dist: %lf, follow_dist: %lf, diff: %lf", dist, mFollowDistance, mFollowDistance - dist);
	double abs_dist = fabs(mFollowDistance - dist);
	double linear = abs_dist < LINEAR_TOLERANCE ? 0.0 : (std::min(mMaxSpeedDistance, abs_dist) / mMaxSpeedDistance) * mMaxLinearSpeed;
	if (dist < mFollowDistance)
		linear *= -1.0;
	//double linear = dist <= mFollowDistance ? 0.0 : (std::max(dist, mMaxSpeedDistance - mFollowDistance) / (mMaxSpeedDistance - mFollowDistance)) * mMaxLinearSpeed;

	double angle = -atan2(mPersonLoc.x, mPersonLoc.y);
	double abs_angle = fabs(angle);
	//ROS_INFO("angle: %lf", angle);
	double angular = abs_angle < ANGULAR_TOLERANCE ? 0.0 : (std::min(abs_angle, mMaxSpeedAngle) / mMaxSpeedAngle) * mMaxAngularSpeed;
	if (angle < 0.0)
		angular *= -1;

	sendSpeed(linear, angular);
	mFollowing = false;
}

void PersonFollower::spin()
{
	ros::Rate rate(50);
	while (ros::ok())
	{
		ros::spinOnce();

		if (mFollowing)
			updateFollow();

		rate.sleep();
	}
}

int main(int argc, char *argv[])
{
	// init ros
	ros::init(argc, argv, "PersonFollower");

	PersonFollower pf;
	pf.spin();

	return 0;
}
