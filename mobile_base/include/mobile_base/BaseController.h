#ifndef __BASECONTROLLER_H
#define __BASECONTROLLER_H

#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/Joy.h>
#include <mobile_base/sensorFeedback.h>
#include <mobile_base/tweak.h>
#include <mobile_base/MotorHandler.h>
#include <numeric>
#include <vector>
#include <math.h>
#include <iostream>

#define EMERGENCY_STOP_DISTANCE 	60	// [cm]
#define MAX_ANGULAR_AT_TOP_SPEED	0.5 // [m/s]
#define MAX_ANGULAR_AT_LOW_SPEED	1.5 // [m/s]
#define MAX_LINEAR_SPEED			0.5 // [m/s]
#define	ULTRASONE_ALL				true
#define	ULTRASONE_NONE				false

enum PS3Key
{
	PS3_NONE                = -1,
	PS3_SELECT				= 0,
	PS3_LEFT_JOYSTICK		= 1,
	PS3_RIGHT_JOYSTICK		= 2,
	PS3_START				= 3,
	PS3_UP                  = 4,
	PS3_RIGHT               = 5,
	PS3_DOWN                = 6,
	PS3_LEFT                = 7,
	PS3_L2                  = 8,
	PS3_R2                  = 9,
	PS3_L1                  = 10,
	PS3_R1                  = 11,
	PS3_T                   = 12,
	PS3_O                   = 13,
	PS3_X                   = 14,
	PS3_S                   = 15,
	PS3_HOME				= 16,
};

enum PS3Axes
{
	PS3_AXIS_LEFT_HORIZONTAL     = 0,
	PS3_AXIS_LEFT_VERTICAL       = 1,
	PS3_AXIS_RIGHT_HORIZONTAL    = 2,
	PS3_AXIS_RIGHT_VERTICAL      = 3,
};

enum PS3ControlMode
{
	PS3_CONTROL_GAME,				/// Resembles how vehicles are driven in (most) games. X is throttle, left joystick is steering. Angular velocity is limited depending on the current linear velocity.
	PS3_CONTROL_REMOTE_CONTROL,		/// Resembles how (most) remote controlled vehicles are driven (left joystick is left motor, right joystick is right motor). Angular velocity is limited the limit of the individual motor (MAX_LINEAR_SPEED).
	PS3_CONTROL_TOTAL,
};

/// Controller for teleoperation
class BaseController
{
protected:
	ros::NodeHandle mNodeHandle;     		/// ROS node handle

	ros::Subscriber mSpeed_sub;		 		/// Listens to Twist messages as feedback for robot's current speed
	ros::Subscriber mKey_sub;        		/// Key input subscriber, reads key input data

	ros::Publisher mMotorControl_pub;       /// Twist message publisher, publishes movement data for engines
	ros::Publisher mTweak_pub;       		/// Integer message publisher, publishes integers for the DPAD buttons

	PS3Key mKeyPressed;  					/// Remembers the last pressed button
	geometry_msgs::Twist mCurrentSpeed;		/// Holds current speed of the robot

	PS3ControlMode mControlMode;			/// Type of control mode for the PS3 controller. Default is PS3_CONTROL_GAME.

	float mPrevAngular;
	float mPrevLeftSpeed;
	float mPrevRightSpeed;

public:
	/// Constructor
	BaseController() : mNodeHandle(""), mControlMode(PS3_CONTROL_GAME), mPrevAngular(0.f), mPrevLeftSpeed(0.f), mPrevRightSpeed(0.f) { }

	/// Destructor
	~BaseController()
	{

		mNodeHandle.shutdown();
	}

	/// Initialize node
	void init();

	/// Listen to PS3 controller
	void keyCB(const sensor_msgs::Joy& msg);

	/// Read current speed for feedback control
	void readCurrentSpeed(const geometry_msgs::Twist& msg);

	//inline double calcRobotAngularSpeed() { if (mCurrentSpeed.linear.x) return (1.0/mCurrentSpeed.linear.x*10.0) * SPEED_CONST;  else return 0.f; };
	inline double calcRobotAngularSpeed() { return MAX_ANGULAR_AT_LOW_SPEED - ((MAX_ANGULAR_AT_LOW_SPEED - MAX_ANGULAR_AT_TOP_SPEED) * mCurrentSpeed.linear.x) / MAX_LINEAR_SPEED; };

	inline ros::NodeHandle* getNodeHandle() { return &mNodeHandle; };
};

#endif /* __CONTROLLER_H */
