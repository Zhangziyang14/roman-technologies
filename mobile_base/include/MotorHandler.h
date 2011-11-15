#ifndef __MOTORHANDLER_H
#define __MOTORHANDLER_H

#include <ros/ros.h>
#include <Motor.h>
#include <mobile_base/BaseMotorControl.h>
#include <sensor_msgs/Joy.h>
#include <mobile_base/tweak.h>
#include <mobile_base/DisableMotor.h>
#include <mobile_base/BumperDisableMotor.h>
#include <BaseController.h>
#include <std_msgs/Float64.h>
#include <boost/thread.hpp>

#define WHEEL_RADIUS 0.1475
#define BASE_RADIUS 0.25
#define PID_TWEAK_STEP 0.01

/// Listens to motor commands and handles them accordingly.
class MotorHandler
{
protected:
    ros::NodeHandle mNodeHandle;		/// ROS node handle

   ros::Subscriber mBumperDisableSub;		// Listens to messages send by the bumpersensors that disables the movement
    ros::Subscriber mDisableSub;		/// Listens to messages that disables the movement
    ros::Subscriber mTwistSub;			/// Listens to Twist messages for movement
    ros::Subscriber mPositionSub;		/// Listens to integer messages for positioning
    ros::Subscriber mTweakPIDSub;		/// Listens to Int messages, the integers represent the pressed DPAD button on the PS3 controller

    ros::Publisher mSpeedPub;			/// Publishes robot's speed

    geometry_msgs::Twist mCurrentSpeed;	/// Twist message containing the current speed of the robot
    double mLeftMotorSpeed;				/// Current speeds for left wheel in rad/s
    double mRightMotorSpeed;				/// Current speeds for right wheel in rad/s

    Motor mLeftMotor;					/// Motor for left wheel
    Motor mRightMotor;					/// Motor for right wheel

    MotorId mMotorId;					/// Id of the motor that is currently being controlled
    PIDParameter mPIDFocus;				/// One of the PID parameters that is to be changed on button events

    bool mDisableForwardMotion;				/// If true, forward motions are disabled by ultrasone
    bool mDisableBackwardMotion;			/// If true, backward motions are disabled by ultrasone

    bool mBumperDisableForwardMotion;			/// If true, forward motions are disabled by bumpers
    bool mBumperDisableBackwardMotion;			/// If true, backward motions are disabled by bumpers

public:
    /// Constructor
    MotorHandler() : mNodeHandle("~"), mLeftMotor(MID_LEFT), mRightMotor(MID_RIGHT), mMotorId(MID_LEFT), mPIDFocus(PID_PARAM_I),
		mDisableForwardMotion(false), mDisableBackwardMotion(false), mBumperDisableForwardMotion(false), mBumperDisableBackwardMotion(false) { }
    
    /// Destructor
    /** Delete motor interface, close serial port, and shut down node handle */
    ~MotorHandler()
    {     
        mNodeHandle.shutdown();
    }

    /// Initialise node
    void init(char *path);

    void publishRobotSpeed();
    void moveCB(const mobile_base::BaseMotorControl& msg);
    void positionCB(const std_msgs::Float64& msg);
    void tweakCB(const mobile_base::tweak msg);
    void checkMotorConnections(char* path);

    inline double getBaseRadius()
    {
    	if (abs(mLeftMotorSpeed) + abs(mRightMotorSpeed) == 0)
    		return 0.0;
    	return BASE_RADIUS * 2 * std::max(abs(mLeftMotorSpeed), abs(mRightMotorSpeed)) / (abs(mLeftMotorSpeed) + abs(mRightMotorSpeed));
    };

    void stopMotor();
    void disableMotorBumperCB(const mobile_base::BumperDisableMotor &msg);
    void disableMotorCB(const mobile_base::DisableMotor &msg);
};    

#endif /* __MOTORHANDLER_H */
