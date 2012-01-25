/*
 * LocalPlanner.h
 *
 *  Created on: 2012-01-20
 *      Author: wilson
 */

#ifndef LOCALPLANNER_H_
#define LOCALPLANNER_H_

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <mobile_base/SensorFeedback.h>

#define WHEEL_RADIUS 							0.1475	//[m]
#define BASE_RADIUS 							0.25	//[m]
#define PID_TWEAK_STEP 							0.01

#define ZERO_SPEED								0.0		//[m/s]
#define STANDARD_AVOIDANCE_IMPACT				150.0

#define FRONT_AVOIDANCE_DISTANCE				150.0	//[cm]
#define FRONT_AVOIDANCE_IMPACT					70.0

#define SIDES_AVOIDANCE_IMPACT					75.0
#define SIDES_AVOIDANCE_DISTANCE				60.0	//[cm]

#define FRONT_SIDES_AVOIDANCE_DISTANCE 			100.0	//[cm]
#define FRONT_CENTER_AND_SIDES_AVOIDANCE_IMPACT	120.0

#define REAR_HALTING_DISTANCE					60.0	//[cm]

class LocalPlanner
{

private:
	ros::Subscriber mUltrasoneSub;			/// Subscriber that listens to ultrasone messages
	ros::Subscriber mSpeedSub;				/// subscriber that listens to speed feedback

	ros::Publisher mSpeedPub;				/// Publishes velocity commands

	boost::array<int16_t, 10> mSensorData;
	geometry_msgs::Twist mCurrentSpeed;


	void speedCB(const geometry_msgs::Twist& msg);
	void ultrasoneCB(const mobile_base::SensorFeedback& msg);

	//Scale the speed of a motor based on the measured distance
	inline double scaleSpeed(double speed, double avoidance_impact, double avoidance_distance, int measured_distance)
	{
		return std::min(speed, speed-speed*(avoidance_distance-measured_distance)/avoidance_impact);
	};

	//Scale the speed of a motor based on the measured distances, using two distances
	inline double scaleSpeed(double speed, double avoidance_impact, double avoidance_distance, int measured_distance1, int measured_distance2)
	{
		return std::min(speed, std::min(speed-speed*(avoidance_distance-measured_distance1)/avoidance_impact, speed-speed*(avoidance_distance-measured_distance2)/avoidance_impact));
	};

	//Scale the speed of a motor based on the measured distances, using four distances
	inline double scaleSpeed(double speed, double avoidance_impact, double avoidance_distance, int distances[])
	{
		return std::min(scaleSpeed(speed, avoidance_impact, avoidance_distance, distances[0], distances[1]), scaleSpeed(speed, avoidance_impact, avoidance_distance, distances[2], distances[3]));

	};

public:
	void scaleTwist(geometry_msgs::Twist& msg);

	LocalPlanner(ros::NodeHandle *nodeHandle);
	~LocalPlanner() {};
};


#endif /* LOCALPLANNER_H_ */
