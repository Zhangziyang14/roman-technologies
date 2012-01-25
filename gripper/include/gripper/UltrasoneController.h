/*
 * UltrasoneController.h
 *
 *  Created on: Oct 11, 2011
 *      Author: hans
 */

#ifndef ULTRASONECONTROLLER_H_
#define ULTRASONECONTROLLER_H_

#include "Controller.h"
#include <std_msgs/Bool.h>

class UltrasoneController: public Controller
{
protected:
	ros::Publisher mPingCommand_pub;	// Sends commands to Ping sensor
	ros::Subscriber mCommand_sub;		/// Listens to commands to open/close gripper
public:
	virtual void init();

	void readSensorDataCB(const std_msgs::UInt16& msg);
	void commandCB(const std_msgs::Bool& msg);
};

#endif /* ULTRASONECONTROLLER_H_ */
