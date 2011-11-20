// Head.pd
// Controls colors and movement of the eyebrows and head
// Author: Ingmar Jager, EEMCS, Delft University of Technology


#include "ros.h"
#include "std_msgs/UInt8.h"
#include "Emotion.h"

ros::NodeHandle nh;


//RGB pins
int redPin = 3;
int greenPin = 5;
int bluePin = 6;

// Available emotions/states
enum Emo
{
  NEUTRAL = 0,
  HAPPY,
  SAD,
  SURPRISED,
  ERROR  
};

//RGB Values for NEURTRAL(r,g,b)
const Emotion neutral(255,255,255);
//RGB Values for Happy
const Emotion happy(255,0,255);
//RGB Values for SAD
const Emotion sad(120,255,0);
//RGB Values for SURPRISED
const Emotion surprised(0,127,255);
//RGB Values for ERROR
const Emotion error(0,255,255);

Emotion currentEmotion(0,0,0);

void expressEmotion(const std_msgs::UInt8 &msg)
 {
 switch(msg.data)
 {
   case NEUTRAL:
     {
       transition(neutral);
     }
   case HAPPY:
     {
       transition(happy);
     }
   case SAD:
     {
       transition(sad);
     }
   case SURPRISED:
     {
       transition(surprised);
     }
   case ERROR:
     {
       transition(error); 
     }
   default:
     {
       transition(neutral);
     }
 }
 
}

//subscribe to emotionTopic
ros::Subscriber<std_msgs::UInt8> emotion_sub("/emotionTopic", &expressEmotion);


void setup()
{
  nh.initNode();
  nh.subscribe(emotion_sub);
  
  //currentEmotion = neutral;
  transition(neutral);
  transition(happy);
  transition(surprised);
  transition(neutral);
}


void loop()
{
  nh.spinOnce();
}

void transition(Emotion nextEmotion)
{
//calculate transition values
  int r = nextEmotion.red() - currentEmotion.red(); 
  int g = nextEmotion.green() - currentEmotion.green(); 
  int b = nextEmotion.blue() - currentEmotion.blue(); 
//transition colors to next state
  double rStep = r/50.0;
  double gStep = g/50.0;
  double bStep = b/50.0;

  for (int i = 0; i<50; i++)
  {
    analogWrite(redPin, currentEmotion.red()+rStep);
    analogWrite(greenPin, currentEmotion.green()+gStep);
    analogWrite(bluePin, currentEmotion.blue()+bStep);
  
    delay(40);  
  }
//set new currentEmotion
  currentEmotion = nextEmotion;

}



