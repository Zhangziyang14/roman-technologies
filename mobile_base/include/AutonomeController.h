/*
 * AutonomeController.h
 *
 *  Created on: Oct 20, 2011
 *      Author: hans
 */

#ifndef AUTONOMECONTROLLER_H_
#define AUTONOMECONTROLLER_H_

#include <mobile_base/sensorFeedback.h>
#include "BaseController.h"

#define EMERGENCY_STOP_DISTANCE 40

/// Controller for autonomous control.
class AutonomeController
{
private:
	ros::NodeHandle mNodeHandle;    		/// ROS node handle
	ros::Subscriber mSensorFeedback_sub; 	/// Subscriber to arduino Ultrasone sensor feedback
	ros::Publisher	mDisableMotor_pub;		/// Publisher for disabling forward/backward movement
public:
	AutonomeController() {};

	void init();

	void ultrasoneFeedbackCB(const mobile_base::sensorFeedback &msg);
};

#endif /* AUTONOMECONTROLLER_H_ */
