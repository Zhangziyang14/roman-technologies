/* Emotion.h - Library for expressing emotions using colors and servo movements
   V0.1 Created by Ingmar Jager, November 20, 2011.
*/


#ifndef Emotion_h
#define Emotion_h

#include <iostream>

class Emotion
{
public:  
  
 Emotion(int r, int g, int b, int left, int right, int lift);
 int red();
 int green();
 int blue();
 int leftEyebrow();
 int rightEyebrow();
 int liftEyebrow();
  
private:
 int _red,_green,_blue;
 int _leftEyebrowAngle, _rightEyebrowAngle;
 int _liftEyebrowAngle;
  
};

#endif
