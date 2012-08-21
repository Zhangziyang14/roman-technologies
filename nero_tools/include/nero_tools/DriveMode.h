/*
 * DriveMode.h
 *
 *  Created on: Aug 17, 2012
 *      Author: hans
 */

#ifndef DRIVEMODE_H_
#define DRIVEMODE_H_

#include "nero_tools/ControllerMode.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose.h"
#include "std_msgs/Bool.h"

#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#define MAX_LINEAR_SPEED 0.8
#define MAX_ANGULAR_SPEED 0.6

#define ARM_POS_DOWN_X (-1.f)
#define ARM_POS_DOWN_Z (-1.f)

#define ARM_POS_UP_X 0
#define ARM_POS_UP_Z (-0.1f)

class DriveMode : public ControllerMode
{
private:
	ros::Publisher mSpeedPub;
	ros::Publisher mArmPosPub;
	ros::Publisher mGripperStatePub;
	ros::Publisher mPingStatePub;

	bool mPingState;
	boost::asio::io_service mIOService;
	boost::asio::deadline_timer mTimer;

	float mLastLinearSpeed;
	float mLastAngularSpeed;

	bool mActive;

public:
	DriveMode(ros::NodeHandle *nodeHandle);

	void handleController(std::vector<int> previousButtons, std::vector<float> previousAxes, const sensor_msgs::Joy &joy);

	void sendSpeed(float linearScale, float angularScale);
	void sendArmPosition(float x, float z);
	void sendGripperState(bool state);
	void sendPingState(bool state);

	void onActivate();
	void onDeactivate();
	void runOnce();

	void sendSpeedEvent();
};


#endif /* DRIVEMODE_H_ */
