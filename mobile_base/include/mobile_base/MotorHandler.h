#ifndef __MOTORHANDLER_H
#define __MOTORHANDLER_H

#include <ros/ros.h>
#include <motors/Motor.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <mobile_base/tweak.h>

#include <mobile_base/position.h>
#include <mobile_base/SensorFeedback.h>

#define WHEEL_RADIUS 			0.1475	//[m]
#define BASE_RADIUS 			0.25	//[m]
#define PID_TWEAK_STEP 			0.01

#define ZERO_SPEED				0.0

#define DISABLE_TRUE			1
#define DISABLE_FALSE			0

/// Listens to motor commands and handles them accordingly.
class MotorHandler
{
protected:
	ros::NodeHandle mNodeHandle;			/// ROS node handle

	ros::Subscriber mTwistSub;				/// Listens to Twist messages for movement
	ros::Subscriber mPositionSub;			/// Listens to integer messages for positioning
	ros::Subscriber mTweakPIDSub;			/// Listens to Int messages, the integers represent the pressed DPAD button on the PS3 controller
    ros::Subscriber mUltrasoneSub;          /// Listens to ultrasone distances

    ros::Publisher mDisableUltrasonePub;	///	Disables unused ultrasone sensors
	ros::Publisher mSpeedPub;				/// Publishes robot's speed

	geometry_msgs::Twist mCurrentSpeed;		/// Twist message containing the current speed of the robot
	double mLeftMotorSpeed;					/// Current speeds for left wheel in rad/s
	double mRightMotorSpeed;				/// Current speeds for right wheel in rad/s

	Motor mLeftMotor;						/// Motor for left wheel
	Motor mRightMotor;						/// Motor for right wheel

	MotorId mMotorId;						/// Id of the motor that is currently being controlled
	PIDParameter mPIDFocus;					/// One of the PID parameters that is to be changed on button events

	bool mLock;

	boost::array<int16_t, 10> mSensorData;

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
	void disableUltrasoneSensors();
	void moveCB(const geometry_msgs::Twist& msg);
	void positionCB(const mobile_base::position& msg);
	void ultrasoneCB(const mobile_base::SensorFeedback& msg);
	void tweakCB(const mobile_base::tweak& msg);

	inline ros::NodeHandle* getNodeHandle() { return &mNodeHandle; };
};

#endif /* __MOTORHANDLER_H */
