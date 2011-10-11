#ifndef __MOTORHANDLER_H
#define __MOTORHANDLER_H

#include <ros/ros.h>
#include <Motor.h>
#include <threemxl/example.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <mobile_base/tweak.h>
#include <threemxl/dxlassert.h>
#include <BaseController.h>

#define WHEEL_RADIUS 0.1475
#define BASE_RADIUS 0.25
#define PID_TWEAK_STEP 0.01

/// Listens to motor commands and handles them accordingly.
class MotorHandler
{
protected:
    ros::NodeHandle mNodeHandle;		/// ROS node handle
    ros::Subscriber mTwistSub;			/// Listens to Twist messages for movement
    ros::Subscriber mTweakPIDSub;		/// Listens to Int messages, the integers represent the pressed DPAD button on the PS3 controller
    ros::Publisher mSpeedSub;			/// Publishes robot's speed
    geometry_msgs::Twist mCurrentSpeed;	/// Twist message containing the current speed of the robot
    double mLeftMotorSpeed;				/// Current speeds for left wheel in rad/s
    double mRightMotorSpeed;			/// Current speeds for right wheel in rad/s

    Motor mLeftMotor;					/// Motor for left wheel
    Motor mRightMotor;					/// Motor for right wheel
    //Motor mArmMotor;					/// Motor for arm

    MotorId mMotorId;					/// Id of the motor that is currently being controlled
    PIDParameter mPIDFocus;				/// One of the PID parameters that is to be changed on button events

public:
    /// Constructor
    MotorHandler() : mNodeHandle("~"), mLeftMotor(MID_LEFT), mRightMotor(MID_RIGHT), mMotorId(MID_LEFT), mPIDFocus(PID_PARAM_I) { }
    
    /// Destructor
    /** Delete motor interface, close serial port, and shut down node handle */
    ~MotorHandler()
    {     
        mNodeHandle.shutdown();
    }

    /// Initialise node
    void init(char *path);

    void publishRobotSpeed();
    void moveCB(const geometry_msgs::Twist& msg);
    void tweakCB(const mobile_base::tweak msg);

    inline double getBaseRadius()
    {
    	if (mLeftMotorSpeed + mRightMotorSpeed == 0)
    		return 0.0;
    	return BASE_RADIUS * 2 * std::max(mLeftMotorSpeed, mRightMotorSpeed) / (mLeftMotorSpeed + mRightMotorSpeed);
    };

};    

#endif /* __MOTORHANDLER_H */
