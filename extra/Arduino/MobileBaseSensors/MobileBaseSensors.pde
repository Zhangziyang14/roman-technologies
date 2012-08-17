//ROS
#include "ros.h"
#include "mobile_base/SensorFeedback.h"
#include <head/Eyebrows.h>
#include <std_msgs/UInt8.h>
#include <mobile_base/disableUltrasone.h>
#include <Wire.h>
#include <Servo.h>
#include <DigiServ.h>
#include <FlexiTimer2.h>

#define EYEBROW_LEFT A3
#define EYEBROW_RIGHT A1
#define EYEBROW_LIFT A0

#define LIFT_LOWER_LIMIT 94
#define LIFT_UPPER_LIMIT 130
#define EYEBROW_LOWER_LIMIT 60
#define EYEBROW_UPPER_LIMIT 120

#define PIN_POT		A0
#define PIN_MOTOR_L	5
#define PIN_MOTOR_R	6

#define SENSOR_ADDRESS(x) (0x70 + x)

using namespace mobile_base;

enum BumperId
{
	BUMPER_UNKNOWN = 0,
	BUMPER_FRONT,
	BUMPER_REAR,
	BUMPER_REAR_LEFT,
	BUMPER_REAR_RIGHT,
	BUMPER_COUNT,
};

const unsigned short bumperPin[BUMPER_COUNT] =
{
	0,		// BUMPER_UNKNOWN
	8,		// BUMPER_FRONT
	9,		// BUMPER_REAR
	10,		// BUMPER_REAR_LEFT
	11,		// BUMPER_REAR_RIGHT
};

Servo eyebrowLeft;
Servo eyebrowRight;

PID pid = {5.0, 0.0, 10.0, 255};
DigiServ digi_lift(PIN_MOTOR_L, PIN_MOTOR_R, PIN_POT, pid);

ros::NodeHandle nh;

//declare outgoing messages
mobile_base::SensorFeedback sensor_msg;
std_msgs::UInt8 bump_msg;

//topic to publish ultrasone sensor data on
ros::Publisher feedback_pub("/sensorFeedbackTopic", &sensor_msg);
//topic that issues a warning when bumper hits something
ros::Publisher bumper_pub("/bumperFeedbackTopic", &bump_msg);

int old_left_eb_angle, old_right_eb_angle, old_lift_angle;
int new_left_eb_angle, new_right_eb_angle, new_lift_angle;
unsigned long start_time;
unsigned int left_eb_angle_time;
unsigned int right_eb_angle_time;
unsigned int lift_time;

//uint16_t enableUltrasoneMask;

void readSensors()
{
	for (int i = 0; i < 2; i++)
	{
		//trigger sensors
		for (int sensor = i; sensor < SensorFeedback::SENSOR_RIGHT; sensor = sensor + 2)
		{
                        //if(enableUltrasoneMask & (1 << sensor))
			  doSRF02Pulse(0x70 + sensor);
		}

		//wait for sound to return
		delay(70);

		//read sensor values
		for (int sensor = i; sensor < SensorFeedback::SENSOR_RIGHT; sensor = sensor + 2)
		{
                        //if(enableUltrasoneMask & (1 << sensor))
			  sensor_msg.data[sensor] = windowFilter(readSRF02(0x70 + sensor));
                        //else
                          //sensor_msg.data[sensor] = SensorFeedback::ULTRASONE_MAX_RANGE;
		}
	}

feedback_pub.publish(&sensor_msg);
delay(100);
}

/*
*  Filters the sensor data within a given window frame
*/
int windowFilter(int data)
{
	if(data > 14 && data < 150)
		return data;
	  
	return 150;
}

/*
 * Trigger sensor to do a burst
 */
void doSRF02Pulse(int address)
{
	Wire.beginTransmission(address); // transmit to device
	Wire.send(0x00);                 // sets register pointer to the command register (0x00)  
	Wire.send(0x51);                 // use 0x51 for measurement in centimeters
	Wire.endTransmission();          // stop transmitting 
}

/*
 * Read address from SRF02 sensor register
 */
int readSRF02(int address)
{
	int range = 0;
	Wire.beginTransmission(address); // transmit to device
	Wire.send(0x02);                 // sets register pointer to echo #1 register (0x02)
	Wire.endTransmission();          // stop transmitting
	// step 4: request reading from sensor
	Wire.requestFrom(address,2);     // request 2 bytes from slave device
	// step 5: receive reading from sensor
	if(2 <= Wire.available())       // if two bytes were received
	{
		range = Wire.receive();        // receive high byte (overwrites previous reading)
		range = range << 8;            // shift high byte to be high 8 bits
		range |= Wire.receive();       // receive low byte as lower 8 bits
	}
	return range; 
}

/*
 * Interrupt routine
 * publishes where the robot has been hit
 */
void bumperHit()
{
	digitalWrite(13, HIGH);

	bump_msg.data = BUMPER_UNKNOWN;
	for (int i = BUMPER_FRONT; i < BUMPER_COUNT; i++)
	{
		if (digitalRead(bumperPin[i]) == 1)
		{
			bump_msg.data = i;
			break;
		}
	}

	bumper_pub.publish(&bump_msg);
	digitalWrite(13, LOW);
}
/*
void disableCB(const mobile_base::disableUltrasone::Request & req, mobile_base::disableUltrasone::Response & res)
{
  enableUltrasoneMask = req.disable;
}*/

/*
ros::ServiceServer<mobile_base::disableUltrasone::Request, mobile_base::disableUltrasone::Response> disableUltrasoneServer("/disableUltrasoneService",&disableCB);
*/
////////////////////////////////////////////////////////////////////////////////////////
//                                  EYEBROW Section
////////////////////////////////////////////////////////////////////////////////////////


/** EYEBROWS
 * Updates eyebrow angles if necessary
 */
void updateEyebrowEvent()
{
	unsigned long now = millis();
	double scale = 0.0;

	int left_eb_angle = 0;
	int right_eb_angle = 0;
	int lift_angle = old_lift_angle;

	// are we set on a timer?
	if (left_eb_angle_time)
	{
		// did our time end?
		if (now - start_time > left_eb_angle_time)
		{
			left_eb_angle_time = 0;
			left_eb_angle = new_left_eb_angle;
		}
		else
		{
			// scale by time passed
			scale = double(now - start_time) / double(left_eb_angle_time);
			left_eb_angle = old_left_eb_angle + scale * (new_left_eb_angle - old_left_eb_angle);
		}

		eyebrowLeft.write(left_eb_angle);
	}

	// are we set on a timer?
	if (right_eb_angle_time)
	{
		// did our time end?
		if (now - start_time > right_eb_angle_time)
		{
			right_eb_angle_time = 0;
			right_eb_angle = new_right_eb_angle;
		}
		else
		{
			// scale by time passed
			scale = double(now - start_time) / double(right_eb_angle_time);
			right_eb_angle = old_right_eb_angle + scale * (new_right_eb_angle - old_right_eb_angle);
		}

		eyebrowRight.write(right_eb_angle);
	}

	// are we set on a timer?
	if (lift_time)
	{
		// did our time end?
		if (now - start_time > lift_time)
		{
			lift_time = 0;
			lift_angle = new_lift_angle;
		}
		else
		{
			// scale by time passed
			scale = double(now - start_time) / double(lift_time);
			lift_angle = scale * (new_lift_angle - old_lift_angle);
		}
	}

        digi_lift.setAngle(lift_angle);
}

/**
 * message order: lift left right
 */
void setEyebrowCB(const head::Eyebrows &msg)
{
	old_left_eb_angle = new_left_eb_angle;
	old_right_eb_angle = new_right_eb_angle;
	old_lift_angle = new_lift_angle;

	new_left_eb_angle = msg.left;
	new_right_eb_angle = msg.right;
	new_lift_angle = msg.lift;

	left_eb_angle_time = max(1, msg.left_time);
	right_eb_angle_time = max(1, msg.right_time);
	lift_time = max(1, msg.lift_time);

        if (new_left_eb_angle > EYEBROW_UPPER_LIMIT)
		new_left_eb_angle = EYEBROW_UPPER_LIMIT;
	else if (new_left_eb_angle < EYEBROW_LOWER_LIMIT)
		new_left_eb_angle = EYEBROW_LOWER_LIMIT;

	if (new_right_eb_angle > EYEBROW_UPPER_LIMIT)
		new_right_eb_angle = EYEBROW_UPPER_LIMIT;
	else if (new_right_eb_angle < EYEBROW_LOWER_LIMIT)
		new_right_eb_angle = EYEBROW_LOWER_LIMIT;  

	if (new_lift_angle > LIFT_UPPER_LIMIT)
		new_lift_angle = LIFT_UPPER_LIMIT;
	else if (new_lift_angle < LIFT_LOWER_LIMIT)
		new_lift_angle = LIFT_LOWER_LIMIT;
	

	start_time = millis();
}

//Subscriber to eyebrow commands from eyebrowTopic
ros::Subscriber<head::Eyebrows> servo_sub("/eyebrowTopic", &setEyebrowCB);

void setup()
{
	//attach interrupt 0(= pin 2) 
	attachInterrupt(0, bumperHit, RISING);
	pinMode(bumperPin[BUMPER_FRONT], INPUT);
	pinMode(bumperPin[BUMPER_REAR], INPUT);
	pinMode(bumperPin[BUMPER_REAR_LEFT], INPUT);
	pinMode(bumperPin[BUMPER_REAR_RIGHT], INPUT);
	//led
	pinMode(13, OUTPUT);

	Wire.begin();

	nh.initNode();

	//eyebrow init
	old_left_eb_angle = 0;
	old_right_eb_angle = 0;
	old_lift_angle = LIFT_LOWER_LIMIT;
	new_left_eb_angle = 0;
	new_right_eb_angle = 0;
	new_lift_angle = LIFT_LOWER_LIMIT;
	left_eb_angle_time = 0;
	right_eb_angle_time = 0;
	lift_time = 0;
	start_time = 0;
	nh.subscribe(servo_sub);
	
        //eyebrowLeft.attach(EYEBROW_LEFT);
	//eyebrowRight.attach(EYEBROW_RIGHT);
	//lift.attach(EYEBROW_LIFT);
	//lift.write(150);
	//delay(500);

	FlexiTimer2::set(150, 1.0/1000, updateEyebrowEvent); // call every 150ms
	FlexiTimer2::start();

	nh.advertise(feedback_pub);
	nh.advertise(bumper_pub);
}

void loop()
{
	readSensors();
	nh.spinOnce(); 
}

/*
*  Returns the measured range in cm of the SRF02 sensor at the given address
*/
/*
int getRange(int address)
{
	Wire.beginTransmission(address); // transmit to device
	Wire.send(0x00);                 // sets register pointer to the command register (0x00)  
	Wire.send(0x51);                 // use 0x51 for measurement in centimeters
	Wire.endTransmission();          // stop transmitting 
	delay(100);

	int range = 0;

	Wire.beginTransmission(address); // transmit to device
	Wire.send(0x02);                 // sets register pointer to echo #1 register (0x02)
	Wire.endTransmission();          // stop transmitting
	// step 4: request reading from sensor
	Wire.requestFrom(address,2);     // request 2 bytes from slave device
	// step 5: receive reading from sensor
	if(2 <= Wire.available())       // if two bytes were received
	{
		range = Wire.receive();        // receive high byte (overwrites previous reading)
		range = range << 8;            // shift high byte to be high 8 bits
		range |= Wire.receive();       // receive low byte as lower 8 bits
	}

	return range; 
}*/

