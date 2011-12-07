/* Emotion.cpp - Library for expressing emotions using colors and servo movements
   V0.1 Created by Ingmar Jager, November 20, 2011.
*/

#include "Emotion.h"
#include "WProgram.h"


Emotion::Emotion(int r, int g, int b)
{
	_red = r;
	_green = g;
	_blue = b;
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
