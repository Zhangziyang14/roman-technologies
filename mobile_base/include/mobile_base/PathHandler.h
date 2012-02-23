/*
 * PathFollower.h
 *
 *  Created on: Jan 20, 2012
 *      Author: hans
 */

#ifndef PATHHANDLER_H_
#define PATHHANDLER_H_

#include "ros/ros.h"
#include "nav_msgs/Path.h"
#include "tf/transform_listener.h"
#include "std_msgs/Float32.h"
#include "std_msgs/UInt8.h"
#include "nav_msgs/Path.h"
#include "sensor_msgs/LaserScan.h"

#define ROBOT_RADIUS 0.4

enum FollowState
{
	FOLLOW_STATE_IDLE,
	FOLLOW_STATE_BUSY,
	FOLLOW_STATE_FINISHED,
	FOLLOW_STATE_MAX,
};

class PathHandler
{
private:
	//LocalPlanner mLocalPlanner;

	ros::Subscriber mPathSub;						/// Subscriber that listens to paths from PathPlanner
	ros::Subscriber mLaserScanSub;
	ros::Publisher mCommandPub;						/// Publishes velocity commands
	ros::Publisher mGoalPub;						/// Re-publishes goals when a new path needs to be calculated
	ros::Publisher mFollowStatePub;					/// Publishes the current state of the follower
	ros::Publisher mLocalPathPub;					/// Publishes a local path

	std::vector<geometry_msgs::Pose> mPath;			/// Current path to follow
	uint32_t mCurrentPathIndex;						/// Index of path we are handling right now
	tf::StampedTransform mRobotPosition;			/// Current position of the robot on the map
	tf::TransformListener mTransformListener;		/// Listens to transforms of the robot

	double mMinAngularSpeed, mMaxAngularSpeed;		/// min-max speed at which to turn, robot will scale depending on the amount of yaw to turn
	double mMinLinearSpeed, mMaxLinearSpeed;		/// min-max linear speed when turning, robot will scale depending on the distance to drive

	FollowState mFollowState;						/// Current state of the follower
	double mFinalYawTolerance;						/// Tolerance while turning before the orientation is accepted
	double mDistanceTolerance;						/// Distance tolerance before a waypoint is reached
	double mResetDistanceTolerance;					/// Distance from robot to path before it will reset its path

	double mLookForwardDistance;
	double mMaxAngleLinearScale;

	inline uint32_t getPathSize() { return mPath.size(); };		/// Returns the size of the remaining path

	inline double getScaledLinearSpeed(double rotationAngle)	/// Scales linear speed based on distance to drive
	{
		double pathLength = getPathLength();
		double scale = (std::max(std::min(pathLength, 3.0), 1.0) - 1.0) / 2.0;

		double angular_scale = 1.0 - std::min(1.0, (fabs(rotationAngle) / mMaxAngleLinearScale)); // Also scale linear by how much we are rotating

		return (scale * (mMaxLinearSpeed - mMinLinearSpeed) + mMinLinearSpeed) * angular_scale;
	};

	inline double getScaledAngularSpeed(double rotationAngle, bool minAngular = false)	/// Scales the angular speed based on the turn to be made
	{
		double scale = std::min(1.0, (fabs(rotationAngle) / mMaxAngleLinearScale));
		double result = scale * (mMaxAngularSpeed - (minAngular ? mMinAngularSpeed : 0.0)) + (minAngular ? mMinAngularSpeed : 0.0);
		return rotationAngle < 0.0 ? -1 * result : result;
	};

	double calculateDiffYaw();

	/// Listen to Twist messages and stream them
	void pathCb(const nav_msgs::Path &path);
	void clearPath();
	bool updateCurrentPosition();
	void publishState();
	void publishLocalPath(double amplitude, double yaw);

	void scanCb(const sensor_msgs::LaserScan &scan);

	double angleTo(geometry_msgs::Point p);
	geometry_msgs::Point getPointOnPathWithDist(geometry_msgs::Point p, double dist);
	double getPathLength();

	inline void resendGoal()
	{
		if (getPathSize() == 0)
		{
			ROS_ERROR("No path, cannot resend goal.");
			return;
		}

		geometry_msgs::PoseStamped msg;
		msg.header.frame_id = "/map";
		msg.header.stamp = ros::Time::now();
		msg.pose = mPath[getPathSize() - 1];
		mGoalPub.publish(msg);
	};

public:
	void updatePath();

	PathHandler(ros::NodeHandle *nodeHandle);
	~PathHandler() {};
};

#endif /* PATHHANDLER_H_ */
