#ifndef __MOTOR_H
#define __MOTOR_H

#include <CDxlGeneric.h>
#include <threemxl/C3mxlROS.h>

#define DEFAULT_P 0.01
#define DEFAULT_I 0.0
#define DEFAULT_D 0.0

#define DEFAULT_ACCELERATION 5
#define SAFE_BRAKING_DECCELERATION 0.5

enum ControlMode
{
    CM_NONE = -1,
	CM_POSITION_MODE,					// uses one motor and encoder with position controller
	CM_SPEED_MODE,						// uses one motor and encoder with speed controller
	CM_CURRENT_MODE,					// uses one motor and encoder with current controller
	CM_TORQUE_MODE,						// uses one motor and encoder with torque controller
	CM_SEA_MODE,						// uses one motor and two encoders with SEA controller
	CM_PWM_MODE,						// uses one motor and encoder with PWM controller, no PID, just PWM !!

	CM_EXT_INIT_MODE = EXTERNAL_INIT,	// reset counter on external io pulse, turn motor with given torque in M3XL_DESIRED_TORQUE_L

    CM_STOP_MODE = 12,					// do nothing, no actuated motors
    CM_TEST_MODE,       				// for general testing
};

enum PIDParameter
{
	PID_PARAM_NONE = -1,
	PID_PARAM_P,
	PID_PARAM_I,
	PID_PARAM_D,
	PID_PARAM_MAX,
};

enum MotorId
{
	MID_NONE = -1,
	MID_LEFT = 106,
	MID_RIGHT = 107,
	MID_GRIPPER = 109,
	MID_ARM_SHOULDER = 110,
	MID_ARM_SIDEWAYS = 111,
};

/// Class that contains PID values and some perhaps limited functions
class PID
{
public:
	PID() : p(0.0), i(0.0), d(0.0) {}
	PID(double proportional, double integral, double derivative) : p(proportional), i(integral), d(derivative) {}

	double &operator[](PIDParameter p);

	double p;
	double i;
	double d;
};

/// Listens to motor commands and handles them accordingly.
class Motor
{
protected:
    MotorId mMotorId;      /// Motor id (left, right or arm motor)
    CDxlGeneric *motor_;    /// Motor interface
    LxSerial serial_port_;  /// Serial port interface
    ControlMode cmode;      /// Keeps track of what ControlMode the motor is in at the moment 

public:
    /// Constructor
    Motor(MotorId id) : mMotorId(id), motor_(NULL), cmode(CM_NONE) { }
    
    /// Destructor
    /** Delete motor interface, close serial port, and shut down node handle */
    ~Motor()
    {
        if (motor_)
            delete motor_;
        if (serial_port_.is_port_open())
            serial_port_.port_close();
    }

    /// Initialize node
    /** \param path path to shared_serial node */
    void init(char *path);

    void setMode(ControlMode mode);
    void setSpeed(double speed);
    void brake();
    void setLSpeed(double speed, double acceleration);
    void setPosition(double position);
    void setAcceleration(double aceleration);
    void updatePID();
    void printPID();
    double getPosition();

    // angular position control
    void setAngle(double angle);
    void setAngle(double angle, double speed);
    void setAngle(double angle, double speed, double accel);
    double getAngle();

    // torque control
    void setTorque(double torque);

    // change turning direction
    void setPositiveDirection(bool clockwise);
    void setAngleLimits(double lowerLimit, double upperLimit);

    double getRotationSpeed();
    int getMode();
    int update();
    int getLog();
    int getID();
    int ping();

    int getStatus();
    inline bool checkPort(){ return serial_port_.is_port_open(); };

    PID mPID;

private:
    void assertMode(ControlMode mode);

};    

#endif /* __MOTOR_H */
