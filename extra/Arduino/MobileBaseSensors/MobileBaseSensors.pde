//ROS
#include "ros.h"
#include "mobile_base/sensorFeedback.h"
#include "std_msgs/UInt8.h"
#include <head/eyebrows.h>
#include <std_msgs/Int32.h>
#include <Wire.h>
#include <Servo.h>

const int eblift = A0;
const int ebright = A1;
const int ebleft = A3;

Servo eyebrowLeft;
Servo eyebrowRight;
Servo lift;
boolean attache;

int bumperFrontPin = 8;
int bumperRearPin = 9;
int bumperLeftPin = 10;
int bumperRightPin = 11;

int ranges[10];

const int LIFT_LOWER_LIMIT = 47;
const int LIFT_UPPER_LIMIT = 75;

const int EYEBROW_LOWER_LIMIT = 60;
const int EYEBROW_UPPER_LIMIT = 120;

// Ultrasone mask flags
enum UltrasoneSensor
{                             //Corresponding hardware address
  SENSOR_FRONT_LEFT = 0,      //0x70
  SENSOR_FRONT_LEFT_CENTER,   //0x71
  SENSOR_FRONT_CENTER_LEFT,   //0x72
  SENSOR_FRONT_CENTER_RIGHT,  //0x73
  SENSOR_FRONT_RIGHT_CENTER,  //0x74
  SENSOR_FRONT_RIGHT,         //0x75
  SENSOR_RIGHT,               //0x76
  SENSOR_REAR_RIGHT,          //0x77
  SENSOR_REAR_LEFT,           //0x78
  SENSOR_LEFT,                //0x79
};

enum BumperId
{
  BUMPER_UNKNOWN = 0,
  BUMPER_FRONT,
  BUMPER_REAR,
  BUMPER_REAR_LEFT,
  BUMPER_REAR_RIGHT
};

ros::NodeHandle nh;

//declare outgoing messages
mobile_base::sensorFeedback prox_msg;
std_msgs::UInt8 bump_msg;

//topic to publish ultrasone sensor data on
ros::Publisher feedback_pub("/sensorFeedbackTopic", &prox_msg);
//topic that issues a warning when bumper hits something
ros::Publisher bumper_pub("/bumperFeedbackTopic", &bump_msg);


void readSensors()
{
      for (int address = 0x70; address < 0x72; address++)
    {
      //trigger sensors
      for (int i = 0; i < 9; i = i + 2)
      {
         doSRF02Pulse(address+i);
      }
      //wait for sound to return
      delay(70);
      
      //read sensor values
      for (int i = 0; i < 9; i = i + 2)
      {
         ranges[address+i-0x70] = windowFilter(readSRF02(address+i));
      }
      
    }
    
   prox_msg.frontLeft = ranges[SENSOR_FRONT_LEFT];                  
   prox_msg.frontLeftCenter = ranges[SENSOR_FRONT_LEFT_CENTER];
   prox_msg.frontCenterLeft = ranges[SENSOR_FRONT_CENTER_LEFT];
   prox_msg.frontCenterRight = ranges[SENSOR_FRONT_CENTER_RIGHT];   
   prox_msg.frontRightCenter = ranges[SENSOR_FRONT_RIGHT_CENTER];
   prox_msg.frontRight = ranges[SENSOR_FRONT_RIGHT];
   prox_msg.right = ranges[SENSOR_RIGHT];  
   prox_msg.rearRight = ranges[SENSOR_REAR_RIGHT];
   prox_msg.rearLeft = ranges[SENSOR_REAR_LEFT];  
   prox_msg.left = ranges[SENSOR_REAR_LEFT];
  
  
   feedback_pub.publish(&prox_msg);
  
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
  if (digitalRead(bumperFrontPin) == 1)
    bump_msg.data = BUMPER_FRONT;
  else if (digitalRead(bumperRearPin) == 1)
    bump_msg.data = BUMPER_REAR;
  else if (digitalRead(bumperLeftPin) == 1)
    bump_msg.data = BUMPER_REAR_LEFT;
  else if (digitalRead(bumperRightPin) == 1)
    bump_msg.data = BUMPER_REAR_RIGHT;
  else
    bump_msg.data = BUMPER_UNKNOWN;
    
  bumper_pub.publish(&bump_msg);
  digitalWrite(13, LOW);
}

////////////////////////////////////////////////////////////////////////////////////////
//                                  EYEBROW Section
////////////////////////////////////////////////////////////////////////////////////////


/** EYEBROWS
 * Checks for angle limits and sets servos to given angles
 */
void setPosition(int liftAngle, int eyebrowLeftAngle, int eyebrowRightAngle)
{
   if (liftAngle > LIFT_UPPER_LIMIT)
     liftAngle = LIFT_UPPER_LIMIT;
   else if (liftAngle < LIFT_LOWER_LIMIT)
      liftAngle = LIFT_LOWER_LIMIT;
     
    if (eyebrowLeftAngle > EYEBROW_UPPER_LIMIT)
     eyebrowLeftAngle = EYEBROW_UPPER_LIMIT;
   else if (eyebrowLeftAngle < EYEBROW_LOWER_LIMIT)
      eyebrowLeftAngle = EYEBROW_LOWER_LIMIT;
    
    if (eyebrowRightAngle > EYEBROW_UPPER_LIMIT)
     eyebrowRightAngle = EYEBROW_UPPER_LIMIT;
   else if (eyebrowRightAngle < EYEBROW_LOWER_LIMIT)
      eyebrowRightAngle = EYEBROW_LOWER_LIMIT;  

   eyebrowLeft.write(eyebrowLeftAngle);
   eyebrowRight.write(eyebrowRightAngle);
   
   if(!attache)
   {
     lift.attach(eblift);
     attache=true;
   }
     
   lift.write(liftAngle);
   
   if(liftAngle < 170)
   {
     lift.detach();
     attache = false;
   }
}

/**
 * message order: lift left right
 */
void setEyebrowCB(const head::eyebrows &msg)
{
   setPosition(msg.lift, msg.left, msg.right);
  
}


//Subscriber to eyebrow commands from eyebrowTopic
ros::Subscriber<head::eyebrows> servo_sub("/eyebrowTopic", &setEyebrowCB);

void setup()
{
  //attach interrupt 0(= pin 2) 
  attachInterrupt(0, bumperHit, RISING);
  pinMode(bumperFrontPin, INPUT);
  pinMode(bumperRearPin, INPUT);
  pinMode(bumperLeftPin, INPUT);
  pinMode(bumperRightPin, INPUT);
  //led
  pinMode(13, OUTPUT);
  
  Wire.begin();
 
  nh.initNode();
  
  //eyebrow init
  nh.subscribe(servo_sub);
  eyebrowLeft.attach(ebleft);
  eyebrowRight.attach(ebright);
  lift.attach(eblift);
  lift.write(150);
  delay(500);
  lift.detach();
  attache = false;
  
  nh.advertise(feedback_pub);
  nh.advertise(bumper_pub);
};

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

