/* Emotion.h - Library for expressing emotions using colors and servo movements
   V0.1 Created by Ingmar Jager, November 20, 2011.
*/


#ifndef Emotion_h
#define Emotion_h

#include "WProgram.h"

class Emotion
{
public:  
  
 Emotion(int r, int g, int b);
 int red();
 int green();
 int blue();
  
private:
 int _red,_green,_blue;
  
};

#endif
