/* Emotion.cpp - Library for expressing emotions using colors and servo movements
   V0.1 Created by Ingmar Jager, November 20, 2011.
*/

#include "head/Emotion.h"

Emotion::Emotion(int r, int g, int b, int left, int right, int lift)
{
	std::cout << "EMOTION" << std::endl; 
	_red 				= r;
	_green 				= g;
	_blue 				= b;
	_leftEyebrowAngle 	= left;
	_rightEyebrowAngle 	= right;
	_liftEyebrowAngle	= lift;
}

int Emotion::leftEyebrow()
{
	return _leftEyebrowAngle;
}

int Emotion::rightEyebrow()
{
	return _rightEyebrowAngle;
}

int Emotion::liftEyebrow()
{
	return _liftEyebrowAngle;
}

int Emotion::red()
{
	return _red;
}

int Emotion::green()
{
	return _green;
}

int Emotion::blue()
{
	return _blue;
}
