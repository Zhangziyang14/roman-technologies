/*
 * PS3Controller.h
 *
 *  Created on: Oct 11, 2011
 *      Author: hans
 */

#ifndef PS3CONTROLLER_H_
#define PS3CONTROLLER_H_

#include "Controller.h"

/// Key ids for PS3 controller (PS3Joy)
enum PS3Key
{
    PS3_NONE = -1,

    PS3_SELECT = 0,
    PS3_LEFT_STICK = 1,
    PS3_RIGHT_STICK = 2,
    PS3_START = 3,

    PS3_UP = 4,
    PS3_RIGHT = 5,
    PS3_DOWN = 6,
    PS3_LEFT = 7,

	PS3_L2 = 8,
	PS3_R2 = 9,
	PS3_L1 = 10,
	PS3_R1 = 11,

	PS3_T = 12,
	PS3_O = 13,
	PS3_X = 14,
    PS3_S = 15,

    PS3_HOME = 16,
};

class PS3Controller: public Controller
{
private:
	ros::Subscriber mKey_sub; 		/// Key input topic, provides key input data (id, value)
	PS3Key mKeyPressed;             /// Contains the current key being pressed (PS3_NONE for no key)
public:
	PS3Controller() : mKeyPressed(PS3_NONE) { }


	virtual void init();

	void keyCB(const sensor_msgs::Joy& msg);
};

#endif /* PS3CONTROLLER_H_ */
