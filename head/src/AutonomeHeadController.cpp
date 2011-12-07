#include "AutonomeHeadController.h"

head::rgb rgb;

void AutonomeHeadController::expressEmotion(const std_msgs::UInt8 &msg)
 {
	

 switch(msg.data)
 {
   case NEUTRAL:
     {
		rgb.r = mNeutral.red();
		rgb.g = mNeutral.green();
		rgb.b = mNeutral.blue();
     }
   case HAPPY:
     {
		rgb.r = mHappy.red();
		rgb.g = mHappy.green();
		rgb.b = mHappy.blue();
     }
   case SAD:
     {
		rgb.r = mSad.red();
		rgb.g = mSad.green();
		rgb.b = mSad.blue();
     }
   case SURPRISED:
     {
		rgb.r = mSurprised.red();
		rgb.g = mSurprised.green();
		rgb.b = mSurprised.blue();
     }
   case ERROR:
     {
		rgb.r = mError.red();
		rgb.g = mError.green();
		rgb.b = mError.blue();
     }
   default:
     {
		rgb.r = mNeutral.red();
		rgb.g = mNeutral.green();
		rgb.b = mNeutral.blue();
     }

	mRGP_pub.Publish(&rgb);
 }
 
	

}
