//ROS
#include "ros.h"
#include "mobile_base/sensorFeedback.h"
#include "std_msgs/UInt8.h"
#include <Wire.h>

byte sensorMask; // Mask containing which sensors should be on or off

int bumperFrontPin = 8;
int bumperRearPin = 9;
int bumperLeftPin = 10;
int bumperRightPin = 11;

// Hardware addresses of the ultrasone sensors
enum HW_ADDRESS
{
   HW_FRONT_LEFT = 0x70,
   HW_FRONT_CENTER = 0x71,
   HW_FRONT_RIGHT = 0x72,
   HW_REAR_RIGHT = 0x73,
   HW_REAR_CENTER = 0x74,
   HW_REAR_LEFT = 0x75,
};

// Ultrasone mask flags
enum UltrasoneSensor
{
  SENSOR_FRONT_LEFT = 0,
  SENSOR_FRONT_CENTER,
  SENSOR_FRONT_RIGHT,
  SENSOR_RIGHT,
  SENSOR_REAR_RIGHT,
  SENSOR_REAR_CENTER,
  SENSOR_REAR_LEFT,
  SENSOR_LEFT,
};

enum BumperId
{
  BUMPER_UNKNOWN = 0,
  BUMPER_FRONT,
  BUMPER_REAR,
  BUMPER_REAR_LEFT,
  BUMPER_REAR_RIGHT
};

#define MAKE_FLAG(x) (1 << x)

void activateSensors(const std_msgs::UInt8 &msg)
{
  sensorMask = msg.data;
}

ros::NodeHandle nh;

//declare outgoing messages
mobile_base::sensorFeedback prox_msg;
std_msgs::UInt8 bump_msg;

//topic to publish ultrasone sensor data on
ros::Publisher feedback_pub("/sensorFeedbackTopic", &prox_msg);
//topic that gives bitmasks for activating or deactivating sensors
ros::Subscriber<std_msgs::UInt8> activate_sub("/sensorActivateTopic", &activateSensors);

ros::Publisher bumper_pub("/bumperFeedbackTopic", &bump_msg);


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
  Serial.begin(9600);
  sensorMask = 0;
 
  nh.initNode();
  nh.advertise(feedback_pub);
  nh.subscribe(activate_sub);
  nh.advertise(bumper_pub);
};


void loop()
{
  if (sensorMask & MAKE_FLAG(SENSOR_FRONT_LEFT))
      prox_msg.frontLeft = windowFilter(getRange(HW_FRONT_LEFT));                  
  if (sensorMask & MAKE_FLAG(SENSOR_FRONT_CENTER))
      prox_msg.frontCenter = windowFilter(getRange(HW_FRONT_CENTER));
  if (sensorMask & MAKE_FLAG(SENSOR_FRONT_RIGHT))
      prox_msg.frontRight = windowFilter(getRange(HW_FRONT_RIGHT));
  if (sensorMask & MAKE_FLAG(SENSOR_REAR_RIGHT))
      prox_msg.rearRight = windowFilter(getRange(HW_REAR_RIGHT));
  if (sensorMask & MAKE_FLAG(SENSOR_REAR_CENTER))
     prox_msg.rearCenter = windowFilter(getRange(HW_REAR_CENTER));
  if (sensorMask & MAKE_FLAG(SENSOR_REAR_LEFT))
     prox_msg.rearLeft = windowFilter(getRange(HW_REAR_LEFT));    
    
  if (sensorMask != 0)
   feedback_pub.publish(&prox_msg);
   nh.spinOnce(); 
  delay(1);
};

/*
*  Filters the sensor data within a given window frame
*/
int windowFilter(int data)
{
  if(data > 14 && data < 100)
      return data;
      
  return -2;
}
/*
void doSRF02Pulse(int address)
{
  Wire.beginTransmission(address); // transmit to device
  Wire.send(0x00);                 // sets register pointer to the command register (0x00)  
  Wire.send(0x51);                 // use 0x51 for measurement in centimeters
  Wire.endTransmission();          // stop transmitting 
}

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
*/

/*
*  Returns the measured range in cm of the SRF02 sensor at the given address
*/
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
}

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

/*
*  Returns the measured range in cm of the SRF02 sensor at the given address

void getTwoRanges(int address1, int address2)
{
    doSRF02Pulse(address1);
    doSRF02Pulse(address2);
    delay(65);
    data[address2]
}
*/
