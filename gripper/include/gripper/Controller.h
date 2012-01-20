#ifndef __CONTROLLER_H
#define __CONTROLLER_H

#include <ros/ros.h>
#include <numeric>
#include <sstream>
#include <CDxlGeneric.h>
#include <gripper/MotorControl.h>

#include "sensor_msgs/JointState.h"
#include "sensor_msgs/Joy.h"
#include "std_msgs/Bool.h"
#include "std_msgs/UInt16.h"

#define OPEN_GRIPPER_DISTANCE 100 // mm
#define CLOSE_GRIPPER_DISTANCE 80 // mm
#define AUTOMATIC_GRIPPER_TORQUE 0.02f // Nm
#define GRIPPER_WAIT_TIME 1000 // msec to wait for opening/closing

#define SOFT_MAX_TORQUE 0.05f /// not really THE max, but the max for our usage
#define SOFT_MAX_CURRENT 0.5f /// not really THE max, but the max for our usage

/// Different gripper states.
enum GripperState
{
    GS_NONE = -1,
    GS_OPEN,
    GS_CLOSED,
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
    ros::Publisher mEmotion_pub;

    GripperState mGripperState;     /// Defines the state of the gripper

    bool mSensorActive;

public:
    /// Constructor
    Controller() : mNodeHandle(""), mGripperState(GS_NONE), mSensorActive(true) { }
    
    /// Destructor
    ~Controller()
    {
        mNodeHandle.shutdown();
    }

    /// Updates joints in RViz
    void UpdateJoints();

    /// Initialize node
    virtual void init();

    /// Publish MotorControl message
    void publish(gripper::MotorControl mc);
};    

#endif /* __CONTROLLER_H */
