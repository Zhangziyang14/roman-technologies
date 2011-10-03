#ifndef __BASECONTROLLER_H
#define __BASECONTROLLER_H

#include <ros/ros.h>
#include <Motor.h>

/// Different movement states.
enum MovementState
{
    STOPPED = -1,
    TURNING_LEFT,
    TURNING_RIGHT,
    MOVING_FORWARD,
    MOVING_BACKWARDS,
};


enum PS3Analog
{
/*
    PS3_NONE = 0,
	PS3_BT_L2 = 8,
	PS3_BT_R2 = 9,
	PS3_BT_L1 = 10,
	PS3_BT_R1 = 11,
	PS3_BT_T = 12,
	PS3_BT_O = 13,
	PS3_BT_X = 14,

    PS3_NONE                    = -1,
	PS3_USB_LEFT_HORIZONTAL     = 0,
	PS3_USB_LEFT_VERTICAL       = 1,
	PS3_USB_RIGHT_HORIZONTAL    = 2,
	PS3_USB_RIGHT_VERTICAL      = 3,

    PS3_NONE = 0,
    PS3_USB_LEFT = 7,
    PS3_USB_RIGHT = 9,
    PS3_USB_UP = 8,
    PS3_USB_DOWN = 10,
*/

    PS3_NONE = 0,
	PS3_BT_L2 = 8,
	PS3_BT_R2 = 9,
	PS3_BT_L1 = 10,
	PS3_BT_R1 = 11,
	PS3_BT_T = 12,
	PS3_BT_O = 13,
	PS3_BT_X = 14,

	PS3_USB_L2 = 48,
	PS3_USB_R2 = 49,
	PS3_USB_L1 = 50,
	PS3_USB_R1 = 51,
	PS3_USB_T = 52,
	PS3_USB_O = 53,
	PS3_USB_X = 54,
};

class BaseController
{
protected:
    ros::NodeHandle mNodeHandle;    /// ROS node handle
    PS3Analog mKeyPressed;          /// Contains the current key being pressed (PS3_NONE for no key)
    MovementState mState;           /// Current state of the mobile base
    ros::Subscriber mKey_sub;       /// Key input topic, provides key input data (id, value)

    Motor* leftEngine;
    Motor* rightEngine;
    Motor* armEngine;

public:
    /// Constructor
    BaseController() : mNodeHandle(""), mKeyPressed(PS3_NONE), mState(STOPPED) { }
    
    /// Destructor
    ~BaseController()
    {
        delete leftEngine;
        delete rightEngine;
        delete armEngine;
        mNodeHandle.shutdown();
    }

    /// Initialize node
    void init();

    void moveForward(double speed);
    void moveBackward(double speed);

    void turnLeft(double speed);
    void turnRight(double speed);

//TODO movement for arm

    void brake();
};

#endif /* __CONTROLLER_H */
