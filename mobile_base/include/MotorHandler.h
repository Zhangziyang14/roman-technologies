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
#define BASE_RADIUS	 0.25

/// Listens to motor commands and handles them accordingly.
class MotorHandler
{
protected:
    ros::NodeHandle nh_;				/// ROS node handle
    ros::Subscriber twist_sub;			/// Listens to Twist messages for movement
    ros::Subscriber tweak_sub;			/// Listens to Int messages, the integers represent the pressed DPAD button on the PS3 controller
    ros::Publisher speed_pub;			/// Publishes robot's speed
    geometry_msgs::Twist speed;			/// Twist message containing the current speed of the robot
    double left_speed, right_speed;		/// Current speeds for left and right wheels in rad/s

    Motor mLeftMotor;					/// Motor for left wheel
    Motor mRightMotor;					/// Motor for right wheel
    //Motor arm_engine;					/// Motor for arm

    MotorId mMotorId;					/// Id of the motor that is currently being controlled
    PIDParameter mPIDFocus;				/// One of the PID parameters that is to be changed on button events

public:
    /// Constructor
    MotorHandler() : nh_("~"), mLeftMotor(MID_LEFT), mRightMotor(MID_RIGHT), mMotorId(MID_LEFT), mPIDFocus(PID_PARAM_I) { }
    
    /// Destructor
    /** Delete motor interface, close serial port, and shut down node handle */
    ~MotorHandler()
    {     
        nh_.shutdown();
    }

    /// Initialise node
    void init(char *path);

    void publishRobotSpeed();
    void moveCB(const geometry_msgs::Twist& msg);
    void tweakCB(const mobile_base::tweak msg);

};    

#endif /* __MOTORHANDLER_H */
