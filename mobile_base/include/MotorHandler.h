#ifndef __MOTORHANDLER_H
#define __MOTORHANDLER_H

#include <ros/ros.h>
#include <Motor.h>
#include <mobile_base/BaseMotorControl.h>
#include <sensor_msgs/Joy.h>
#include <mobile_base/tweak.h>
#include <mobile_base/DisableMotor.h>
#include <BaseController.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <mobile_base/sensorFeedback.h>
#include <limits>

//#include <boost/interprocess/sync/interprocess_semaphore.hpp>

#define WHEEL_RADIUS 	0.1475
#define BASE_RADIUS 	0.25
#define PID_TWEAK_STEP 	0.01
#define ULTRASONE_ALL	true
#define ULTRASONE_NONE	false

/// Listens to motor commands and handles them accordingly.
class MotorHandler
{
protected:
    ros::NodeHandle mNodeHandle;		/// ROS node handle

 // ros::Subscriber mDisableSub;		/// Listens to messages that disables the movement
    ros::Subscriber mTwistSub;			/// Listens to Twist messages for movement
    ros::Subscriber mPositionSub;		/// Listens to integer messages for positioning
    ros::Subscriber mUltrasoneSub;		/// Listens to distance from ultrasone sensors
    ros::Subscriber mTweakPIDSub;		/// Listens to Int messages, the integers represent the pressed DPAD button on the PS3 controller
    ros::Subscriber mDummySub;			/// Listens to dummy velocity commands

    ros::Publisher mSpeedPub;			/// Publishes robot's speed
    ros::Publisher mUltrasoneActivatePub; 	/// Publishes messages to activate ultrasone sensors

    geometry_msgs::Twist mCurrentSpeed;	 	/// Twist message containing the current speed of the robot
    double mLeftMotorSpeed;			/// Current speeds for left wheel in rad/s
    double mRightMotorSpeed;			/// Current speeds for right wheel in rad/s

    Motor mLeftMotor;				/// Motor for left wheel
    Motor mRightMotor;				/// Motor for right wheel

    MotorId mMotorId;				/// Id of the motor that is currently being controlled
    PIDParameter mPIDFocus;			/// One of the PID parameters that is to be changed on button events

    bool mLock;

    bool mDisableForwardMotion;		/// If true, forward motions are disabled by ultrasone
    bool mDisableBackwardMotion;		/// If true, backward motions are disabled by ultrasone
    int mFrontLeftCenter, mFrontRightCenter, mRear, mLeft, mRight, mFrontLeft, mFrontRight, mRearRight, mRearLeft;

public:
    /// Constructor
    MotorHandler() : mNodeHandle("~"), mLeftMotor(MID_LEFT), mRightMotor(MID_RIGHT), mMotorId(MID_LEFT), mPIDFocus(PID_PARAM_I) { }
		//mDisableForwardMotion(false), mDisableBackwardMotion(false) { }
    
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
    void dummyCB(const std_msgs::Float64& msg);
    void ultrasoneCB(const mobile_base::sensorFeedback& msg);
    void tweakCB(const mobile_base::tweak msg);
    void checkMotorConnections(char* path);

    inline double getBaseRadius()
    {
    	if (abs(mLeftMotorSpeed) + abs(mRightMotorSpeed) == 0)
    		return 0.0;
    	return BASE_RADIUS * 2 * std::max(abs(mLeftMotorSpeed), abs(mRightMotorSpeed)) / (abs(mLeftMotorSpeed) + abs(mRightMotorSpeed));
    };

   void disableMotorCB(const mobile_base::DisableMotor &msg);

};

#endif /* __MOTORHANDLER_H */
