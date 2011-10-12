//ROS
#include "ros.h"
#include "mobile_base/sensorFeedback.h"
#include "mobile_base/activateSensors.h"

//I2C
#include <Wire.h>


boolean activationState[7];
int data[6];
long pingData;

const int led = 13;
const int sensorPin = 7;

void activateSensors(const mobile_base::activateSensors &msg)
{
  //pinMode(led, OUTPUT);
  //digitalWrite(led, HIGH);
  activationState[0] = msg.fr;
  activationState[1] = msg.fl;
  activationState[2] = msg.cr;
  activationState[3] = msg.cl;
  activationState[4] = msg.rr;
  activationState[5] = msg.rl;
  activationState[6] = msg.ping;
}

//declare incomming messages
mobile_base::sensorFeedback prox_msg;

ros::NodeHandle  nh;
ros::Publisher feedback_pub("/sensorFeedbackTopic", &prox_msg);
ros::Subscriber<mobile_base::activateSensors> activate_sub("/sensorActivateTopic", &activateSensors);




/**
* setup() gets called at startup.
*  Initiates the Two Wire Interface and the ROS node.
*/
void setup()
{
 Wire.begin();
 memset(data,-1, sizeof(data));   // set all elements of data to -1 
 memset(activationState, false, sizeof(activationState));
 nh.initNode();
 nh.advertise(feedback_pub);
 nh.subscribe(activate_sub);
  
}

/**
* RunLoop first 
*/
void loop()
{
  
  //Read PING sensor if active
  if (activationState[6])
  {
   doPulse(); // start super sonic burst
   long duration = readPulse(); // duration in microseconds
   pingData = microsecondsToMilimeters(duration); //distance in milimeters
  }
  else
    pingData = -1;
  
  for (int i = 0; i < 6; i++)
  {
    if (activationState[i])
      data[i] = getRange(0x70+i); 
    else data[i] = -1;    // When sensors is not activated
  }
  
  if (active()){
  dataToMessage();
  publishProximityData();
  }
  nh.spinOnce();
  delay(1);
  
}

/**
* Publishes the sensor data to ROS
*/
void publishProximityData()
{
 feedback_pub.publish(&prox_msg);
}


int getRange(int address)
{
 int range = 0;
 // step 1: instruct sensor to read echoes
  Wire.beginTransmission(address); // transmit to device (#112 (0x70))
                                   // the address specified in the datasheet is 224 (0xE0)
                                   // but i2c adressing uses the high 7 bits so it's 112 (address/2)
  Wire.send(0x00);                 // sets register pointer to the command register (0x00)  
  Wire.send(0x51);                 // use 0x51 for measurement in centimeters
  Wire.endTransmission();          // stop transmitting
// step 2: wait for readings to happen
  delay(65);                       // datasheet suggests at least 65 milliseconds
// step 3: instruct sensor to return a particular echo reading
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

/**
* Put data in message
*/
void dataToMessage()
{
  prox_msg.frontLeft = data[0];   // 0x70 | 0xE0 | 112 
  prox_msg.frontRight = data[1];  // 0x71 | 0xE1 | 113
  prox_msg.centerLeft = data[2];  // 0x72 | 0xE2 | 114
  prox_msg.centerRight = data[3]; // 0x73 | 0xE3 | 115
  prox_msg.rearLeft = data[4];    // 0x74 | 0xE4 | 116
  prox_msg.rearRight = data[5];   // 0x75 | 0xE5 | 117
  prox_msg.ping = pingData;       //Ping Sensor 32 bit int
}


/**
* Start sending out an ultrasonic pulse from PING
*/
void doPulse(){
  pinMode(sensorPin, OUTPUT);
  digitalWrite(sensorPin, LOW);
  delayMicroseconds(2);
  digitalWrite(sensorPin, HIGH);
  delayMicroseconds(5);
  digitalWrite(sensorPin, LOW); 
}  

/**
* Reading is done from same pin as the output pin, so change pinMode. a HIGH
* pulse whose duration is the time (in microseconds) from the sending
* of the ping to the reception of its echo off an object.
*/
long readPulse(){   
  pinMode(sensorPin, INPUT);
  return pulseIn(sensorPin, HIGH); 
}

/**
* Convert measured time to distance;
*/
long microsecondsToMilimeters(long microseconds)
{ 
  //speed of sound = 343.2 metres per second. Or 2.913752914 µs per mm. 
   return microseconds / 2.913 / 2; 
}

/**
* True if at least one sensor is active
*/
boolean active()
{
  return (activationState[0] | activationState[1] | activationState[2] | activationState[3] | 
          activationState[4] | activationState[5] | activationState[6]);
}

