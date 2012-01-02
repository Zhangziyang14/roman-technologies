/*
 * PathFinder.cpp
 *
 *  Created on: 2011-11-11
 *      Author: wilson
 */

#include "PathFinder.h"

using namespace boost::numeric::ublas;

ros::Time current_time, last_time;

/*
 * Returns a translated vector
 * @param target: vector to be translated
 *
 * @param horizontal: horizontal translation
 *
 * @param vertical: vertical translation
 */
vector<double> PathFinder::translateVector(vector<double> target, double horizontal, double vertical)
{
	identity_matrix<double> identity (3);
	matrix<double> translationMatrix (3,3);

	//Specify homogeneous coordinates
	target.resize(3, true);
	target.insert_element(X3, 1);

	//Create translation matrix with correct parameters horizontal translation and vertical translation
	translationMatrix.insert_element(ROW_1, COLUMN_3, horizontal);
	translationMatrix.insert_element(ROW_2, COLUMN_3, vertical);
	translationMatrix += identity;

	//Translate the target vector
	vector<double> result = prod(translationMatrix, target);

	//Convert homogeneous coordinates back to ordinary coordinates
	result.resize(2, true);

	return result;
}

/*
 * Calculates coordinates of the vector in a rotated coordinate system
 * @param target: vector with coordinates in the original coordinate system
 *
 * @param theta: angle of rotation
 */
vector<double> PathFinder::rotateCoordinateAxes(vector<double> target, double theta)
{
	//Standard rotation matrix (inverted) for rotation of coordinate axes
	matrix<double> rotationMatrix(2, 2);
	rotationMatrix.insert_element(ROW_1, COLUMN_1, cos(theta));
	rotationMatrix.insert_element(ROW_1, COLUMN_2, sin(theta));
	rotationMatrix.insert_element(ROW_2, COLUMN_1, -sin(theta));
	rotationMatrix.insert_element(ROW_2, COLUMN_2, cos(theta));

	target = prod(rotationMatrix, target);

	return target;
}

/*
 *Calculates current angle of the robot in original coordinate system
 */
void PathFinder::readCurrentAngleCB(const geometry_msgs::Twist& msg)
{
	current_time 		 = ros::Time::now();
	double angular_speed = msg.angular.z;
	double dt 			 = (current_time - last_time).toSec();
	mCurrentAngle 		 =  angular_speed * dt;
	last_time 			 = current_time;
}

/*
 * Calculates current position of the robot in original coordinate system
 */
void PathFinder::readCurrentPosCB(const nav_msgs::Odometry& msg)
{
	double x_coord = msg.pose.pose.position.x;
	double y_coord = msg.pose.pose.position.y;

	mCurrent->insert_element(X1, x_coord);
	mCurrent->insert_element(X2, y_coord);
}

/*
 * Calculates the coordinates of the target in the robot's coordinate system
 * Also publishes the angle to be turned and distance to be driven to get to target.
 */
void PathFinder::convertTargetBaseCB(const logical_unit::target& msg)
{
	double x_coord = msg.x;
	double y_coord = msg.y;

	//Create target vector from coordinates
	mTarget->insert_element(X1, x_coord);
	mTarget->insert_element(X2, y_coord);

	//Give the target vector in coordinates of the robot's coordinate system
	//Translate robot's coordinate system to origin, this is equal to translating the target vector with the negative coordinates of the robot
	vector<double> result = translateVector(*mTarget, ((vector<double>) *mCurrent)(X1)*-1, ((vector<double>) *mCurrent)(X2)*-1);

	//Specify vector with coordinates in the rotated coordinate system
	result = rotateCoordinateAxes(result, mCurrentAngle);

	logical_unit::path path_message;

	//TODO Move calculation to Controller, send result vector to Controller instead

	//Calculate distance from robot to target
	double distance = sqrt((double) inner_prod(result, result));

	//Calculate angle from robot's front to target
	double angle	= std::atan((double) result(X2) / result(X1));

	path_message.distance 	= distance;
	path_message.angle 		= angle;

	//Publish the distance to drive and angle to turn to reach target
	mPath_pub.publish(path_message);
}

/*
 * Initialises Pathfinder
 */
void PathFinder::init()
{
	//initialise subscribers
	mCurrent_sub = mNodeHandle.subscribe("odom", 10, &PathFinder::readCurrentPosCB, this);
	mTwist_sub	 = mNodeHandle.subscribe("/speedFeedbackTopic", 1, &PathFinder::readCurrentAngleCB, this);
	mTarget_sub  = mNodeHandle.subscribe("pathTopic", 10, &PathFinder::convertTargetBaseCB, this);

	//initialise publishers
	mPath_pub	 = mNodeHandle.advertise<logical_unit::path>("/processedPathTopic", 10);

	//initialise target and current vectors
	mTarget 		= new vector<double>(2);
	mCurrent 		= new vector<double>(2);
	mCurrentAngle   = 0;

	ROS_INFO("Path finder initialised");
}

int main(int argc, char **argv)
{
	// init ros and pathfinder
	ros::init(argc, argv, "path_finder");
	PathFinder pathFinder;

	char *path = NULL;
	if (argc == 2)
		path = argv[1];

	pathFinder.init();
	ros::spin();
}
