/*
 * Roman Technologies 
 * - Head rgb/servo control
 * - Ping readout
 * - 1 publisher: /pingFeedbackTopic
 * - 3 subscribers: /rgbTopic /servoTopic /pingActivateTopic
 * Author: Ingmar Jager
 */

#include <ros.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Empty.h>
#include <head/panTilt.h>
#include <head/eyebrows.h>
#include <head/rgb.h>
#include <head/breath.h>
#include <TimerOne.h>
#include <Servo.h>

//Breath properties
double transition_speed = 0.5;
double breath_speed = 0.05;
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
const int bluePin = 3;
const int ebleft = A4;
const int ebright = A2;
const int eblift = A3;
const int tiltPin = A1;
const int panPin = A5;
const int pingPin = 8;

Servo eyebrowLeft;
Servo eyebrowRight;
Servo lift;
Servo panServo;
Servo tiltServo;

const int LIFT_LOWER_LIMIT = 0;
const int LIFT_UPPER_LIMIT = 30;

const int EYEBROW_LOWER_LIMIT = 0;
const int EYEBROW_UPPER_LIMIT = 45;

const int PAN_UPPER_LIMIT = 160;
const int PAN_LOWER_LIMIT = 20;

const int TILT_UPPER_LIMIT = 70;
const int TILT_LOWER_LIMIT = 40;

//ping properties
boolean pingActivated;
std_msgs::UInt16 ping_msg;


//ROS
ros::NodeHandle  nh;
ros::Publisher ping_pub("/pingFeedbackTopic", &ping_msg);


////////////////////////////////////////////////////////////////////////////////////////
//                                   RGB Section
////////////////////////////////////////////////////////////////////////////////////////

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
  else // take a breath
  {
    if (countDown)
       count--;
    else
       count++;
    
    int newRed = red + (count * breath_speed);
    int newGreen = green + (count * breath_speed);
    int newBlue = blue + (count * breath_speed);
    
    if ((newRed > 40) && (newRed > red * 0.5) && newRed < targetRed + 1)
       red = newRed;
    if ((newGreen > 40) && (newGreen > green * 0.5) && (newGreen < targetGreen + 1))
       green = newGreen;
    if ((newBlue > 40) && (newBlue > blue * 0.5) && (newBlue < targetBlue +1))
       blue = newBlue;
    
    //Pause at low brightness
    if (count < -1*low_pause)
    {
       countDown = false;
       count = 0;
    }
    
    // Pause at full brightness
    if (count > full_pause)
    {
       countDown = true;
       count = 0;
    }
    
  }
  
   analogWrite(redPin, red);
   analogWrite(greenPin, green);
   analogWrite(bluePin, blue);
  
}

/**
 * sets designated color targets received through rgbTopic
 */
void setRGBCB(const head::rgb &msg)
{
  targetRed = msg.red;
  targetGreen = msg.green;
  targetBlue = msg.blue;
 // breathInterval = msg.a;
  changeState = true;
}

void setBreathPropertiesCB(const head::breath &msg)
{
  breath_speed = msg.breathSpeed;
  transition_speed = msg.transitionSpeed;
  full_pause = msg.fullPause;
  low_pause = msg.lowPause; 
}

ros::Subscriber<head::rgb> rgb_sub("/rgbTopic", &setRGBCB);
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
 * wrapper for setPosition
 */
void setEyebrowCB(const head::eyebrows &msg)
{
  setPosition(msg.lift, msg.left, msg.right);
}

ros::Subscriber<head::eyebrows> servo_sub("/eyebrowTopic", &setEyebrowCB);

////////////////////////////////////////////////////////////////////////////////////////
//                                  PAN_TILT Section
////////////////////////////////////////////////////////////////////////////////////////





void panTiltCB(const head::panTilt &msg)
{
  
}

ros::Subscriber<head::panTilt> servo2_sub("/panTiltTopic", &panTiltCB);

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
  
  //pantTilt init
  nh.subscribe(servo2_sub);
  panServo.attach(panPin);
  tiltServo.attach(tiltPin);
  
  //ping init
  pingActivated = false;
  nh.advertise(ping_pub);
  nh.subscribe(ping_sub);
  
  //timer
  Timer1.initialize(15000);         // initialize timer1, and set a 15000 us period
  Timer1.attachInterrupt(transition);  // attaches breath() as a timer overflow interrup\
  
}

void loop()
{
  ping();
  nh.spinOnce();
}

