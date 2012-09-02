/*
 * PS3Mode.h
 *
 *  Created on: Aug 17, 2012
 *      Author: hans
 */

#ifndef PS3MODE_H_
#define PS3MODE_H_

#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "std_msgs/UInt8.h"
#include "nero_msgs/Emotion.h"
#include "nero_msgs/SpeechCommand.h"
#include "nero_msgs/PitchYaw.h"
#include "geometry_msgs/Point.h"

enum PS3Key
{
    PS3_NONE = -1,

    PS3_SELECT,
    PS3_LEFT_STICK,
    PS3_RIGHT_STICK,
    PS3_START,

    PS3_UP,
    PS3_RIGHT,
    PS3_DOWN,
    PS3_LEFT,

	PS3_L2,
	PS3_R2,
	PS3_L1,
	PS3_R1,

	PS3_T,
	PS3_O,
	PS3_X,
    PS3_S,

    PS3_HOME,

    PS3_BUTTON_MAX,
};

enum PS3Axis
{
    PS3_AXIS_NONE = -1,

    PS3_AXIS_LEFT_HORIZONTAL,
    PS3_AXIS_LEFT_VERTICAL,
    PS3_AXIS_RIGHT_HORIZONTAL,
    PS3_AXIS_RIGHT_VERTICAL,

    PS3_AXIS_UP,
    PS3_AXIS_RIGHT,
    PS3_AXIS_DOWN,
    PS3_AXIS_LEFT,

	PS3_AXIS_L2,
	PS3_AXIS_R2,
	PS3_AXIS_L1,
	PS3_AXIS_R1,

	PS3_AXIS_T,
	PS3_AXIS_O,
	PS3_AXIS_X,
    PS3_AXIS_S,

    PS3_AXIS_RIGHT_LEFT,
    PS3_AXIS_FORWARD_BACKWARD,
    PS3_AXIS_UP_DOWN,
    PS3_AXIS_ROTATION,
};

#define HEAD_SLEEP_PITCH 0.8f
#define HEAD_SLEEP_YAW 0

#define HEAD_AWAKE_PITCH 0
#define HEAD_AWAKE_YAW 0

class ControllerMode
{
private:
	ros::NodeHandle *mNodeHandle;

	ros::Publisher mEmotionPub;
	ros::Publisher mSpeechCommandPub;
	ros::Publisher mHeadPosPub;
	ros::Publisher mPersonPosPub;

	bool mAwake;

public:
	ControllerMode(ros::NodeHandle *nodeHandle);

	virtual void handleController(std::vector<int> previousButtons, std::vector<float> previousAxes, const sensor_msgs::Joy &joy);
	virtual void onActivate() {};
	virtual void onDeactivate() {};
	virtual void runOnce() {};

	void sendEmotion(uint8_t emotion);
	void sendSpeechCommand(std::string command);
	void sendHeadPosition(float pitch, float yaw);
	void sendPersonFollowing(int x, int y);

	bool pressed(std::vector<int> previousButtons, const sensor_msgs::Joy &joy, PS3Key key);
};

#endif /* PS3MODE_H_ */
