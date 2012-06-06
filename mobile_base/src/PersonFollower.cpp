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
	mNodeHandle.param<double>("/PersonFollower/max_speed_distance", mMaxSpeedDistance, 3.0);
	mNodeHandle.param<double>("/PersonFollower/max_speed_angle", mMaxSpeedAngle, 1.0);
}

void PersonFollower::positionCb(const geometry_msgs::PointStampedPtr &point)
{
	if (point->header.frame_id != "/base_link")
	{
		ROS_ERROR("Person position is nog in /base_link!");
		return;
	}

	mPersonLoc = point->point;
	mFollowing = true;
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
	// calculate the distance to the person
	double dist = sqrt(mPersonLoc.x*mPersonLoc.x + mPersonLoc.y*mPersonLoc.y + mPersonLoc.z*mPersonLoc.z);

	if (dist > MAX_DISTANCE)
	{
		ROS_ERROR("PersonFollower distance is larger than the maximum distance!");
		return;
	}

	double linear = dist <= mFollowDistance ? 0.0 : (std::max(dist, mMaxSpeedDistance - mFollowDistance) / (mMaxSpeedDistance - mFollowDistance)) * mMaxLinearSpeed;
	double angle = atan2(mPersonLoc.y, mPersonLoc.x);
	double angular = angle < ANGULAR_TOLERANCE ? 0.0 : (std::max(angle, mMaxSpeedAngle) / mMaxSpeedAngle) * mMaxAngularSpeed;

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
