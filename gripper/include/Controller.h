#ifndef __CONTROLLER_H
#define __CONTROLLER_H

#include <ros/ros.h>
#include <numeric>
#include <sstream>
#include <Distance.h>
#include <CDxlGeneric.h>
#include <MotorControl.h>

#include "sensor_msgs/JointState.h"
#include "sensor_msgs/Joy.h"
#include "std_msgs/Empty.h"

/// Different gripper states.
enum GripperState
{
    GS_NONE = -1,
    GS_OPEN,
    GS_CLOSED,
};

/// Key ids for bluetooth connected PS3 controller (PS3Joy) and USB connected PS3 controller
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



/// Receives sensor feedback and sends gripper control messages.
class Controller
{
protected:
    ros::NodeHandle mNodeHandle;    /// ROS node handle

    ros::Publisher mSensor_pub;     /// Sensor topic, used for toggling the sensor
    ros::Publisher mMotor_pub;      /// Motor topic, used for controlling the motor
    ros::Publisher mJoint_pub;      /// Joint topic, used for changing the joints in RViz

    ros::Subscriber mSensor_sub;    /// Sensor feedback topic, provides sensor data
    ros::Subscriber mKey_sub;       /// Key input topic, provides key input data (id, value)

    GripperState mGripperState;     /// Defines the state of the gripper
    PS3Key mKeyPressed;             /// Contains the current key being pressed (PS3_NONE for no key)

public:
    /// Constructor
    Controller() : mNodeHandle(""), mGripperState(GS_NONE), mKeyPressed(PS3_NONE) { }
    
    /// Destructor
    ~Controller()
    {
        mNodeHandle.shutdown();
    }

    /// Updates joints in RViz
    void UpdateJoints();

    /// Initialize node
    void init();

    /// Publish MotorControl message
    void publish(gripper::MotorControl mc);

    void keyCB(const sensor_msgs::Joy& msg);
    void readSensorDataCB(const gripper::DistancePtr& msg);
};    

#endif /* __CONTROLLER_H */
