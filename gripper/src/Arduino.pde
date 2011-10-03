#include <ros.h>
#include <std_msgs/Empty.h>
#include <roman/Distance.h>
#include <roman/Temperature.h>

const int sensorPin = 7;
const int led = 13;
const int TMP36 = A0;

roman::Distance dis_msg;
roman::Temperature temp_msg;  

boolean activateSensor = false;

/**
* Wait for the trigger to activate sensor
*/
void activateSensorCB(const std_msgs::Empty& toggle_msg){
  activateSensor = !activateSensor;
}

ros::NodeHandle  nh;
ros::Subscriber<std_msgs::Empty> sub("/roman/sensorTopic", &activateSensorCB);
ros::Publisher ultrasone_pub("/roman/sensorFeedbackTopic", &dis_msg);
ros::Publisher temperature_pub("/roman/temperatureFeedbackTopic", &temp_msg);

/**
* Arduino setup, initialize Node
*/
void setup(){
  nh.initNode();
  nh.advertise(ultrasone_pub);
  nh.advertise(temperature_pub);
  nh.subscribe(sub);    
  pinMode(led, OUTPUT);
  pinMode(TMP36, INPUT);
}

/**
* Start sending out an ultrasonic pulse
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
* Publish the ultrasone data to the sensorFeedbackTopic 
*/
void uploadUltrasoneData(){
   if (activateSensor == false){
    digitalWrite(led, LOW);
    return;
   }
  
  digitalWrite(led, HIGH);

  doPulse();
  int duration = readPulse();
  dis_msg.distance = microsecondsToMilimeters(duration);
  printf("Distance: %d", dis_msg.distance);
  ultrasone_pub.publish(&dis_msg); 
}

/**
* Publish temperature data to the temperatureFeedbackTopic
*/
void uploadTemperatureData(){

 //10mV/C => 2.048 bits/10mV => 2.048 bits / C Offset = 0.5 V
 int sensorValue = analogRead(A0)/2.048 - 50;
 temp_msg.temperature = sensorValue;
 temperature_pub.publish(&temp_msg);
}

/**
* Keep sending out sensordata
*/
void loop(){ 
  uploadUltrasoneData();
  uploadTemperatureData();
  nh.spinOnce();
  delay(1);
}
