/*
 * PersonFollower.h
 *
 *  Created on: Jun 5, 2012
 *      Author: hans
 */

#ifndef PERSONFOLLOWER_H_
#define PERSONFOLLOWER_H_

#include "ros/ros.h"

#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/Twist.h"

#define MAX_DISTANCE 5.0 // in meters

#define LINEAR_TOLERANCE 0.1
#define ANGULAR_TOLERANCE 0.1

class PersonFollower
{
private:
	ros::NodeHandle mNodeHandle;
	ros::Publisher mSpeedPub;
	ros::Subscriber mPersonPosSub;

	geometry_msgs::Point mPersonLoc;

	bool mFollowing;

	double mMaxLinearSpeed;
	double mMaxAngularSpeed;
	double mFollowDistance;
	double mMaxSpeedDistance;
	double mMaxSpeedAngle;

public:
	PersonFollower();

	void positionCb(const geometry_msgs::PointStampedPtr &point);

	void spin();

	void updateFollow();
	void sendSpeed(double linear, double angular);
};


#endif /* PERSONFOLLOWER_H_ */
