/*
 * AutonomeMobileController.h
 *
 *  Created on: 2012-01-20
 *      Author: wilson
 */

#ifndef AUTONOMEMOBILECONTROLLER_H_
#define AUTONOMEMOBILECONTROLLER_H_


#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <mobile_base/MotorHandler.h>

#include "nav_msgs/Path.h"
#include "tf/transform_listener.h"
#include "std_msgs/Float32.h"
#include "std_msgs/UInt8.h"

enum FollowState
{
	FOLLOW_STATE_IDLE,
	FOLLOW_STATE_TURNING,
	FOLLOW_STATE_FORWARD,
	FOLLOW_STATE_FINISHED,
	FOLLOW_STATE_MAX,
};


/// Controller for teleoperation
class AutonomeMobileController
{
protected:
	ros::NodeHandle mNodeHandle;      /// ROS node handle

	ros::Publisher mSpeedPub; 	      /// Twist message publisher, publishes movement data for engines

private:
	ros::Subscriber mPathSub;						/// Subscriber that listens to paths from PathPlanner
	ros::Publisher mCommandPub;						/// Publishes velocity commands
	ros::Publisher mGoalPub;						/// Re-publishes goals when a new path needs to be calculated
	ros::Publisher mPathLengthPub;					/// Publishes path length when path is refreshed
	ros::Publisher mFollowStatePub;					/// Publishes the current state of the follower
	std::list<geometry_msgs::Pose> mPath;			/// Current path to follow
	geometry_msgs::Point mOrigin;					/// Point the robot originated from when moving to a next waypoint
	tf::StampedTransform mRobotPosition;			/// Current position of the robot on the map
	tf::TransformListener mTransformListener;		/// Listens to transforms of the robot

	int mRefreshRate;								/// Rate at which the path gets updated
	double mMinAngularSpeed, mMaxAngularSpeed;		/// min-max speed at which to turn, robot will scale depending on the amount of yaw to turn
	double mAngularAdjustmentSpeed;					/// Angular speed while driving to adjust for offsets
	double mMinLinearSpeed, mMaxLinearSpeed;		/// min-max linear speed when turning, robot will scale depending on the distance to drive

	FollowState mFollowState;						/// Current state of the follower
	double mYawTolerance;							/// Tolerance before turning correctly while driving forward
	double mFinalYawTolerance;						/// Tolerance while turning before the orientation is accepted
	double mDistanceTolerance;						/// Tolerance in distance for considering a waypoint as reached
	double mResetDistanceTolerance;					/// Distance from robot to path before it will reset its path

	inline geometry_msgs::Pose getGoal() { return mPath.back(); };					/// Returns goal position
	inline geometry_msgs::Point getOrigin() { return mOrigin; };
	inline geometry_msgs::Point getNextPoint() { return mPath.front().position; };	/// Returns current waypoint the robot is getting to
	inline uint32_t getPathSize() { return mPath.size(); };								/// Returns the size of the remaining path
	inline bool reachedNextPoint()													/// Checks if we are close enough to the next point
	{
		btVector3 np(getNextPoint().x, getNextPoint().y, getNextPoint().z);
		return np.distance(mRobotPosition.getOrigin()) < mDistanceTolerance;
	};
	inline double getScaledLinearSpeed()											/// Scales linear speed based on distance to drive
	{
		btVector3 np(getNextPoint().x, getNextPoint().y, getNextPoint().z);
		double scale = (std::max(std::min(np.distance(mRobotPosition.getOrigin()), 3.0), 1.0) - 1.0) / 2.0;
		return scale * (mMaxLinearSpeed - mMinLinearSpeed) + mMinLinearSpeed;
	};
	inline double getScaledAngularSpeed(double rotationAngle)						/// Scales the angular speed based on the turn to be made
	{
		double scale = fabs(rotationAngle) / M_PI;
		return scale * (mMaxAngularSpeed - mMinAngularSpeed) + mMinAngularSpeed;
	};

	double calculateDiffYaw();

	/// Listen to Twist messages and stream them
	void pathCb(const nav_msgs::Path &path);
	void continuePath();
	void clearPath();
	void handlePath();
	bool updateCurrentPosition();
	void publishPathLength();
	void publishState();

public:
	/// Constructor
	AutonomeMobileController();

	/// Destructor
	~AutonomeMobileController()
	{
		mNodeHandle.shutdown();
	};

	void spin();
};

#endif /* AUTONOMEMOBILECONTROLLER_H_ */
