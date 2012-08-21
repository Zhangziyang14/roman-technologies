/*
 * PS3Controller.h
 *
 *  Created on: Aug 17, 2012
 *      Author: hans
 */

#ifndef PS3CONTROLLER_H_
#define PS3CONTROLLER_H_

#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "sensor_msgs/JoyFeedbackArray.h"
#include "std_msgs/UInt16.h"
#include "nero_tools/ControllerMode.h"
#include "nero_tools/DriveMode.h"

#define MAX_MODES 4

#define BATTERY_FULL 1023
#define BATTERY_EMPTY 950

class PS3Controller
{
private:
	ros::NodeHandle mNodeHandle;

	ros::Subscriber mPS3Sub;
	ros::Subscriber mBatteryLogSub;
	ros::Publisher mPS3FeedbackPub;

	ControllerMode* mPS3Modes[MAX_MODES];
	int mPS3ModeCount;
	int mPS3ModeIndex;

	std::vector<int> mPreviousButtons;
	std::vector<float> mPreviousAxes;

	bool mPS3Active;

	inline ControllerMode* getCurrentMode() { return mPS3Modes[mPS3ModeIndex]; };

public:
	PS3Controller();

	void controllerCb(const sensor_msgs::Joy& joy);
	void batteryLogCb(const std_msgs::UInt16& log);

	void addMode(ControllerMode *mode);
	void spin();

	ros::NodeHandle* getNodeHandle() { return &mNodeHandle; };
};


#endif /* PS3CONTROLLER_H_ */
