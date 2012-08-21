/*
 * ArmMode.h
 *
 *  Created on: Aug 21, 2012
 *      Author: parallels
 */

#ifndef ARMMODE_H_
#define ARMMODE_H_

#include "nero_tools/ControllerMode.h"
#include "nero_msgs/ArmJoint.h"

#define MAX_WRIST_SPEED 0.2f
#define MAX_UPPER_SPEED 0.2f

class ArmMode : public ControllerMode
{
private:
	ros::Publisher mArmSpeedPub;

public:
	ArmMode(ros::NodeHandle *nodeHandle);

	void handleController(std::vector<int> previousButtons, std::vector<float> previousAxes, const sensor_msgs::Joy &joy);

	void sendArmSpeed(float wristScale, float upperScale);
};


#endif /* ARMMODE_H_ */
