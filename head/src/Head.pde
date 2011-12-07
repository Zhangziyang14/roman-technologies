/*
 * rosserial Publisher Example
 * Prints "hello world!"
 */

#include <ros.h>
#include <std_msgs/ColorRGBA.h>

enum rgb
{
  RED,
  GREEN,
  BLUE,
};

const int redPin = 9;
const int greenPin = 10;
const int bluePin = 11;
int red, green, blue;
ros::NodeHandle  nh;

void transition(int nextRed, int nextGreen, int nextBlue)
{
 float dR,dG,dB;
 dR = (nextRed - red) / 50;
 dG = (nextGreen - green) / 50;
 dB = (nextBlue - blue) / 50;
 
 for (int i = 0; i < 50; i++)
 {
   red += dR; 
   green += dG;
   blue += dB;
   analogWrite(greenPin, green);
   analogWrite(redPin, red);
   analogWrite(bluePin, blue);
   delay(20);
 }
 
  
}

void setRGB(const std_msgs::ColorRGBA &msg)
{
  transition(msg.r, msg.g, msg.b);
}




ros::Subscriber<std_msgs::ColorRGBA> rgb_sub("/rgbTopic", &setRGB);

void setup()
{
  nh.initNode();
  nh.subscribe(rgb_sub);
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);
  red = 0;
  green = 0;
  blue = 0;
  

  //nh.advertise(chatter);
}




//std_msgs::String str_msg;
//ros::Publisher chatter("chatter", &str_msg);


void loop()
{
  
  for (float i = 1.0; i > 0.5; i = i - 0.01)
  {

   analogWrite(greenPin, green*i);
   analogWrite(redPin, red*i);
   analogWrite(bluePin, blue*i);
   nh.spinOnce();
   delay(20);
  }
  delay(800);
  for (float i = 0.5; i < 1.0; i = i + 0.01)
  {

   analogWrite(greenPin, green*i);
   analogWrite(redPin, red*i);
   analogWrite(bluePin, blue*i);
   nh.spinOnce();
   delay(20);
  }
  delay(1500); 
//  chatter.publish( &str_msg );
  nh.spinOnce();
  delay(10);
}



