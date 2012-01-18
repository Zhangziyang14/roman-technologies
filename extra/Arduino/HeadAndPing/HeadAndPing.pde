/*
 * Roman Technologies 
 * - Head rgb/servo control
 * - Ping readout
 * - 1 publisher: /pingFeedbackTopic
 * - 5 subscribers: /rgbTopic /breathTopic /pingActivateTopic /panTiltTopic /eyebrowTopic
 * Author: Ingmar Jager
 */

#include <ros.h>
#include <std_msgs/UInt16.h>
//#include <std_msgs/UInt8.h>
#include <std_msgs/Empty.h>

#include <std_msgs/ColorRGBA.h>

#include <head/panTilt.h>
#include <head/eyebrows.h>
//#include <head/rgb.h>
#include <head/breath.h>
#include <TimerOne.h>
#include <Servo.h>
#include <FlexiTimer2.h>

//Breath properties
double transition_speed = 3;
double breath_speed = 3000;
int full_pause = 85;
int low_pause = 300;



double red, green, blue; 
int targetRed, targetGreen, targetBlue;
boolean changeState = false;    // if false => breath, if true => transition to new color
boolean countDown = true;      //CLEAN: could be a static boolean inside transition()'s scope
int count = 0;

//Pin definitions
const int redPin = 6;
const int greenPin = 5;
const int bluePin = 9;
const int ebleft = A4;
const int ebright = A2;
const int eblift = A3;
const int panPin = A5;
const int pingPin = 8;

Servo eyebrowLeft;
Servo eyebrowRight;
Servo lift;

const int LIFT_LOWER_LIMIT = 47;
const int LIFT_UPPER_LIMIT = 75;

const int EYEBROW_LOWER_LIMIT = 60;
const int EYEBROW_UPPER_LIMIT = 120;

//ping properties
boolean pingActivated;
std_msgs::UInt16 ping_msg;
//std_msgs::UInt8 dbg_msg;

//ROS
ros::NodeHandle  nh;
ros::Publisher ping_pub("/pingFeedbackTopic", &ping_msg);

//ros::Publisher dbg_pub("/debugTopic", &dbg_msg);

////////////////////////////////////////////////////////////////////////////////////////
//                                   RGB Section
////////////////////////////////////////////////////////////////////////////////////////

    double bRed = red; 
    double bGreen = green; 
    double bBlue = blue;
/**
 * Called every timer interrupt
 * Changes colors fluently
 * Colors fade up and down
 */
void transition()
{
  //transition to new color from current values
  if(changeState)
  {
    if (red != targetRed)
        red += red < targetRed ? transition_speed : (-1*transition_speed);
    if (green != targetGreen)
        green += green < targetGreen ? transition_speed : (-1*transition_speed); 
    if (blue != targetBlue)
        blue += blue < targetBlue ? transition_speed : (-1*transition_speed);   
    if (red == targetRed && green == targetGreen && blue == targetBlue)
        changeState = false;  
        
  }
  /*else // take a breath
  {
    
    float deltaRed = targetRed/ 2.0 / breath_speed;
    float deltaGreen = targetGreen/ 2.0 / breath_speed;
    float deltaBlue = targetBlue/ 2.0 / breath_speed;
   
   
    static boolean dir = true;   

    if ((bRed != targetRed/2.0) && (bGreen != targetGreen/2.0) && (bBlue != targetBlue/2.0) && dir == true)  //decrease brightness
    {
        bRed -= deltaRed;
        bGreen -= deltaGreen;
        bBlue -= deltaBlue;
    }
    else if((bRed != targetRed) && (bGreen != targetGreen) && (bBlue != targetBlue)) //increase brightness
    {
        dir == false;
        bRed += deltaRed;
        bGreen += deltaGreen;
        bBlue += deltaBlue;
    }
    else
    {
      dir == true; //breath again
    }
  }
  
  /*red = bRed;
  green = bGreen;
  blue = bBlue;*/
   /* if (countDown)
       count--;
    else
       count++;
    
    int newRed = red + (count * breath_speed);
    int newGreen = green + (count * breath_speed);
    int newBlue = blue + (count * breath_speed);
    
    if ((newRed > 40) && (newRed > targetRed * 0.45) && newRed < targetRed + 1)
       red = newRed;
    if ((newGreen > 40) && (newGreen > targetGreen * 0.45) && (newGreen < targetGreen + 1))
       green = newGreen;
    if ((newBlue > 40) && (newBlue > targetBlue * 0.45) && (newBlue < targetBlue +1))
       blue = newBlue;
    
    //Pause at low brightness
    if (count*breath_speed < (max(max(targetGreen*0.45, targetRed*0.45),targetBlue*0.45)*-1 - low_pause))
    {
       countDown = false;
       count = 0;
    }
    
    // Pause at full brightness
    if (count*breath_speed > (max(max(targetGreen, targetRed),targetBlue) + full_pause))
    {
       countDown = true;
       count = 0;
    }
    
  }*/

   analogWrite(redPin, red);
   analogWrite(greenPin, green);
   analogWrite(bluePin, blue);
  
}

/**
 * sets designated color targets received through rgbTopic
 */
void setRGBCB(const std_msgs::ColorRGBA &msg)
{
  targetRed = msg.r;
  targetGreen = msg.g;
  targetBlue = msg.b;
  changeState = true;
}

void setBreathPropertiesCB(const head::breath &msg)
{
  breath_speed = msg.breathSpeed;
  transition_speed = msg.transitionSpeed;
  full_pause = msg.fullPause;
  low_pause = msg.lowPause; 
}

//ros::Subscriber<head::rgb> rgb_sub("/rgbTopic", &setRGBCB);
ros::Subscriber<std_msgs::ColorRGBA> rgb_sub("/rgbTopic", &setRGBCB);
ros::Subscriber<head::breath> breath_sub("/breathTopic", &setBreathPropertiesCB);

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
   lift.write(liftAngle); 
  
 
}

/**
 * message order: lift left right
 */
void setEyebrowCB(const std_msgs::ColorRGBA &msg)
{
   setPosition(msg.r,msg.g,msg.b);
  
}

ros::Subscriber<std_msgs::ColorRGBA> servo_sub("/eyebrowTopic", &setEyebrowCB);


////////////////////////////////////////////////////////////////////////////////////////
//                                    ping Section
////////////////////////////////////////////////////////////////////////////////////////


/**
* Start sending out an ultrasonic pulse
*/
void doPulse(){
  pinMode(pingPin, OUTPUT);
  digitalWrite(pingPin, LOW);
  delayMicroseconds(2);
  digitalWrite(pingPin, HIGH);
  delayMicroseconds(5);
  digitalWrite(pingPin, LOW); 
}  

/**
* Reading is done from same pin as the output pin, so change pinMode. a HIGH
* pulse whose duration is the time (in microseconds) from the sending
* of the ping to the reception of its echo off an object.
*/
long readPulse(){   
  pinMode(pingPin, INPUT);
  return pulseIn(pingPin, HIGH); 
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
 * measure and publish ping data if activated
 */
void ping()
{
   if(pingActivated){
      static int pingCount = 0;
       if(pingCount % 901 == 0)
       {
          doPulse();
          int duration = readPulse();
          ping_msg.data = microsecondsToMilimeters(duration); 
          ping_msg.data = (ping_msg.data > 500) ? 500 : ping_msg.data;
          ping_pub.publish(&ping_msg);
       }
       pingCount++;
       if (pingCount > 901)
         pingCount = 0;
    
       }
}

/**
* toggle ping sensor
*/
void activatePingCB(const std_msgs::Empty& toggle_msg){
  pingActivated = !pingActivated;
}


ros::Subscriber<std_msgs::Empty> ping_sub("/pingActivateTopic", &activatePingCB);

////////////////////////////////////////////////////////////////////////////////////////
//                                       setup & loop Section
////////////////////////////////////////////////////////////////////////////////////////



void setup()
{
  nh.initNode();
  
  //rgb init
  nh.subscribe(rgb_sub);
  nh.subscribe(breath_sub);
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);
  red = 0;
  green = 0;
  blue = 0;
  
  //eyebrow init
  nh.subscribe(servo_sub);
  eyebrowLeft.attach(ebleft);
  eyebrowRight.attach(ebright);
  lift.attach(eblift);
  
  //set eyebrows starting position
   eyebrowLeft.write(90);
  eyebrowRight.write(90);
  lift.write(79);
  //nh.advertise(dbg_pub);
  
  //ping init
  pingActivated = false;
  nh.advertise(ping_pub);
  nh.subscribe(ping_sub);
  
  //timer
  //Timer1.initialize(15000);         // initialize timer1, and set a 15000 us period
  //Timer1.attachInterrupt(transition);  // attaches breath() as a timer overflow interrup\
  
   FlexiTimer2::set(15, 1.0/1000, transition); // call every 500 1ms "ticks"
  // FlexiTimer2::set(500, flash); // MsTimer2 style is also supported
   FlexiTimer2::start();

  
}

void loop()
{
  ping();
  nh.spinOnce();
}

