/*
 * UltrasoneController.h
 *
 *  Created on: Oct 11, 2011
 *      Author: hans
 */

#ifndef ULTRASONECONTROLLER_H_
#define ULTRASONECONTROLLER_H_

#include "Controller.h"

class UltrasoneController: public Controller
{
public:
	virtual void init();

	void readSensorDataCB(const gripper::DistancePtr& msg);
};

#endif /* ULTRASONECONTROLLER_H_ */
