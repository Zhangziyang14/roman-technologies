/*
 * PathFinder.h
 *
 *  Created on: 2011-11-11
 *      Author: wilson
 */

#ifndef PATHFINDER_H_
#define PATHFINDER_H_

#include <ros/ros.h>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/io.hpp>
#include <logical_unit/target.h>
#include <logical_unit/path.h>

using namespace boost::numeric::ublas;

enum VectorElements
{
	X1,
	X2,
	X3,
};

enum MatrixRows
{
	ROW_1,
	ROW_2,
	ROW_3,
};

enum MatrixColumns
{
	COLUMN_1,
	COLUMN_2,
	COLUMN_3,
};

class PathFinder{

protected:

	vector<double> *mTarget;		/// Vector specifying the target coordinates in the map
	vector<double> *mCurrent;		/// Vector specifying the current coordinates of the robot in the map
	double mCurrentAngle;			/// Current angle of the robot
	ros::NodeHandle mNodeHandle;    /// ROS node handle
	ros::Subscriber mCurrent_sub;	/// Listens to actual coordinates of the robot
	ros::Subscriber mTarget_sub;	/// Listens to target coordinates
	ros::Subscriber mTwist_sub;		/// Listens to current speed of the robot
	ros::Publisher mPath_pub;     	/// Publishes distances and angles to drive the path

public:
	/// Constructor
	PathFinder() : mNodeHandle(""){}

	/// Destructor
	~PathFinder()
	{
		delete mTarget;
		delete mCurrent;
		mNodeHandle.shutdown();
	}

	void init();
	vector<double> rotateCoordinateAxes(boost::numeric::ublas::vector<double> target, double theta);
	vector<double> translateVector(boost::numeric::ublas::vector<double> target, double horizontal, double vertical);
	void readCurrentPosCB(const nav_msgs::Odometry& msg);
	void readCurrentAngleCB(const geometry_msgs::Twist& msg);
	void convertTargetBaseCB(const logical_unit::target& msg);

};

#endif /* PATHFINDER_H_ */
