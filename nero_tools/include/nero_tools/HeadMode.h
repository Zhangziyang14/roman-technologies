/*
 * ArmMode.h
 *
 *  Created on: Aug 21, 2012
 *      Author: hans
 */

#ifndef HEADMODE_H_
#define HEADMODE_H_

#include "nero_tools/ControllerMode.h"
#include "nero_msgs/PitchYaw.h"

#define MAX_PITCH_SPEED 0.4f
#define MAX_YAW_SPEED 0.4f

class HeadMode : public ControllerMode
{
private:
	ros::Publisher mHeadSpeedPub;

public:
	HeadMode(ros::NodeHandle *nodeHandle);

	void handleController(std::vector<int> previousButtons, std::vector<float> previousAxes, const sensor_msgs::Joy &joy);

	void sendHeadSpeed(float pitchScale, float yawScale);
};


#endif /* HEADMODE_H_ */
