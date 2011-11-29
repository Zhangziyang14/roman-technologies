//ROS
#include "ros.h"
#include "mobile_base/sensorFeedback.h"
#include "mobile_base/activateSensors.h"
#include "gripper/Distance.h"
#include "std_msgs/Bool.h"
#include "mobile_base/activatePing.h"

//I2C
#include <Wire.h>


#define PING  8
boolean activationState[8];
int data[7];

long pingData;


const int sensorPin = 7;



void activateSensors(const mobile_base::activateSensors &msg)
{
  for(int i; i < 9; i++)
  {
      activationState[i] = true;
  }
}

void activatePing(const mobile_base::activatePing &msg)
{
      activationState[PING] = msg.ping; 
}

//declare outgoing messages
mobile_base::sensorFeedback prox_msg;
gripper::Distance dist_msg;

ros::NodeHandle  nh;
ros::Publisher feedback_pub("/sensorFeedbackTopic", &prox_msg);
//ros::Publisher ping_pub("/pingFeedbackTopic", &dist_msg);
ros::Subscriber<mobile_base::activateSensors> activate_sub("/sensorActivateTopic", &activateSensors);
ros::Subscriber<mobile_base::activatePing> ping_sub("/pingActivateTopic", &activatePing);

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
 //nh.advertise(ping_pub);
 nh.subscribe(activate_sub);
 nh.subscribe(ping_sub);
 pinMode(13, OUTPUT);
}

/**
* RunLoop first 
*/
void loop()
{
  
 
  //Read PING sensor if active
  if (activationState[PING])
  {
   doPulse(); // start super sonic burst
   long duration = readPulse(); // duration in microseconds
   pingData = microsecondsToMilimeters(duration); //distance in milimeters
   dist_msg.distance = pingData;        //Ping Sensor 32 bit int
   //ping_pub.publish(&dist_msg);
  }
 
  for (int i = 0; i < 6; i++)
  {
    if(activationState[i])
    {
       
          data[i] = windowFilter(getRange(0x70+i));
          //getRange(0x70+i);
    } 
    else 
        data[i] = -1;    // When sensors are not activated
  }
  
  if (active()){
  dataToMessage();
  feedback_pub.publish(&prox_msg);
  }
  nh.spinOnce();
  delay(1);
}

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
 * Comparison function for quick sort
 
int int_cmp(const void *a, const void *b) {
  int x = *(int*) a, y = *(int*) b;
  return (x == y) ? 0 : (x < y) ? -1 : 1;
}

/**
*  Filters the sensor data

int median_filter(int *data, int len) {
  int median, count, sum, i;
  
  //memcpy(filterData, data, len * sizeof(int));
  qsort(data, len, sizeof(int), int_cmp);
  median = data[len / 2];
  
  count = 0, sum = 0;
  for (i = 0; i < len; ++i) {
    if (data[i] >= 0.5 * median && data[i] <= 1.5 * median) {
      sum += data[i];
      ++count;
    }
  }
  return (int) (0.5 + sum * 1.0 / count);
}
*/

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
  delay(100);                       // datasheet suggests at least 65 milliseconds
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
  prox_msg.frontLeft = data[0];    // 0x70 | 0xE0 | 112 
  prox_msg.frontCenter = data[1];  // 0x71 | 0xE2 | 113
  prox_msg.frontRight = data[2];   // 0x72 | 0xE4 | 114
  prox_msg.rearRight = data[3];    // 0x73 | 0xE6 | 115
  prox_msg.rearCenter = data[4];   // 0x74 | 0xE8 | 116
  prox_msg.rearLeft = data[5];     // 0x75 | 0xEA | 117
  prox_msg.left = data[6];         // Infrared Left
  prox_msg.right = data[7];        // Infrared Right
  prox_msg.ping = pingData;
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
          activationState[4] | activationState[5] | activationState[6] | activationState[7] | activationState[8]);
}

