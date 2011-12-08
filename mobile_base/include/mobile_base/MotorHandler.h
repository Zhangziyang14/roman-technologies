#ifndef __MOTORHANDLER_H
#define __MOTORHANDLER_H

#include <ros/ros.h>
#include <mobile_base/Motor.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <mobile_base/tweak.h>
#include <mobile_base/BaseController.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <mobile_base/sensorFeedback.h>

#define WHEEL_RADIUS 							0.1475	//[m]
#define BASE_RADIUS 							0.25	//[m]
#define PID_TWEAK_STEP 							0.01

#define ULTRASONE_ALL							true	//Activate all ultrasone sensors
#define ULTRASONE_NONE							false	//Deactivate all ultrasone sensors
#define ULTRASONE_MAX_RANGE						600		//[cm]

#define ZERO_SPEED								0.0		//[m/s]
#define STANDARD_AVOIDANCE_IMPACT				175.0	//[cm]

#define FRONT_AVOIDANCE_DISTANCE				150.0	//[cm]
#define FRONT_AVOIDANCE_IMPACT					100.0	//[cm]

#define SIDES_AVOIDANCE_IMPACT					75.0	//[cm]
#define SIDES_AVOIDANCE_DISTANCE				60.0	//[cm]

#define FRONT_SIDES_AVOIDANCE_DISTANCE 			120.0	//[cm]
#define FRONT_CENTER_AND_SIDES_AVOIDANCE_IMPACT	120.0	//[cm]

/// Listens to motor commands and handles them accordingly.
class MotorHandler
{
protected:
	ros::NodeHandle mNodeHandle;			/// ROS node handle

	ros::Subscriber mTwistSub;				/// Listens to Twist messages for movement
	ros::Subscriber mPositionSub;			/// Listens to integer messages for positioning
	ros::Subscriber mUltrasoneSub;			/// Listens to distance from ultrasone sensors
	ros::Subscriber mTweakPIDSub;			/// Listens to Int messages, the integers represent the pressed DPAD button on the PS3 controller

	ros::Publisher mSpeedPub;				/// Publishes robot's speed
	ros::Publisher mUltrasoneActivatePub; 	/// Publishes messages to activate ultrasone sensors

	geometry_msgs::Twist mCurrentSpeed;		/// Twist message containing the current speed of the robot
	double mLeftMotorSpeed;					/// Current speeds for left wheel in rad/s
	double mRightMotorSpeed;				/// Current speeds for right wheel in rad/s

	Motor mLeftMotor;						/// Motor for left wheel
	Motor mRightMotor;						/// Motor for right wheel

	MotorId mMotorId;						/// Id of the motor that is currently being controlled
	PIDParameter mPIDFocus;					/// One of the PID parameters that is to be changed on button events

	bool mLock;

	int mFrontLeftCenter, mFrontRightCenter, mRear, mLeft, mRight, mFrontLeft, mFrontRight, mRearRight, mRearLeft;

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
	void positionCB(const std_msgs::Float64& msg);
	void ultrasoneCB(const mobile_base::sensorFeedback& msg);
	void tweakCB(const mobile_base::tweak msg);

	//Scale the speed of a motor based on the measured distance
	inline double scaleSpeed(double speed, double avoidance_impact, double avoidance_distance, int measured_distance)
	{
		return std::min(speed, std::min(speed-speed*(avoidance_distance-measured_distance)/avoidance_impact, speed-speed*(avoidance_distance-measured_distance)/avoidance_impact));
	};

	//Scale the speed of a motor based on the measured distances
	inline double scaleSpeed(double speed, double avoidance_impact, double avoidance_distance, int measured_distance1, int measured_distance2)
	{
		return std::min(speed, std::min(speed-speed*(avoidance_distance-measured_distance1)/avoidance_impact, speed-speed*(avoidance_distance-measured_distance2)/avoidance_impact));
	};
};

#endif /* __MOTORHANDLER_H */
