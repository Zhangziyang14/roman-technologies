//ROS
#include "ros.h"
#include <Servo.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt16.h>
#include <FlexiTimer2.h>
#include <nero_msgs/Eyebrows.h>

#define batteryThreshold 512 //FIXME
#define EYEBROW_LEFT 8
#define EYEBROW_RIGHT 9

#define EYEBROW_LOWER_LIMIT 60
#define EYEBROW_UPPER_LIMIT 120

int batteryPin = A0;


Servo eyebrowLeft;
Servo eyebrowRight;
int old_left_eb_angle, old_right_eb_angle;
int new_left_eb_angle, new_right_eb_angle;
unsigned long start_time;
unsigned int left_eb_angle_time;
unsigned int right_eb_angle_time;



ros::NodeHandle nh;
std_msgs::UInt16 bat_log;
ros::Publisher bat_log_pub("/batteryLogTopic", &bat_log);

std_msgs::Bool bat_msg;
ros::Publisher bat_pub("/batteryTopic", &bat_msg);

int readBatteryPower()
{
  
     
  bat_msg.data = true; 
  bat_pub.publish(&bat_msg);
   
   return analogRead(batteryPin);

   /*if (adcValue < batteryThreshold) {
     bat_msg.data = true; 
     bat_pub.publish(&bat_msg); 
   }*/
}

int bat_count = 0;

static const int movingAverageSize = 100;
int batteryValues[movingAverageSize];
int index = 0;
void loop()
{
  
  
   if (bat_count > 10000){
     batteryValues[index] = readBatteryPower();
     index = (index+1) % movingAverageSize;
     unsigned long sum = 0;
     int elements = 0;
     for (int i = 0; i < movingAverageSize; i++)
     {
       if (batteryValues[i] != 0) {
         sum += batteryValues[i];
         elements++; 
       }
        
     }
     bat_log.data = sum / elements;
     bat_log_pub.publish(&bat_log);
     bat_count = 0;
   }
   else
     bat_count++;
   nh.spinOnce(); 
  
}

////////////////////////////////////////////////////////////////////////////////////////
//                                  EYEBROW Section
////////////////////////////////////////////////////////////////////////////////////////


/** EYEBROWS
 * Updates eyebrow angles if necessary
 */
void updateEyebrowEvent()
{
	unsigned long now = millis();
	double scale = 0.0;

	int left_eb_angle = 0;
	int right_eb_angle = 0;

	// are we set on a timer?
	if (left_eb_angle_time)
	{
		// did our time end?
		if (now - start_time > left_eb_angle_time)
		{
			left_eb_angle_time = 0;
			left_eb_angle = new_left_eb_angle;
		}
		else
		{
			// scale by time passed
			scale = double(now - start_time) / double(left_eb_angle_time);
			left_eb_angle = old_left_eb_angle + scale * (new_left_eb_angle - old_left_eb_angle);
		}

		eyebrowLeft.write(left_eb_angle);
	}

	// are we set on a timer?
	if (right_eb_angle_time)
	{
		// did our time end?
		if (now - start_time > right_eb_angle_time)
		{
			right_eb_angle_time = 0;
			right_eb_angle = new_right_eb_angle;
		}
		else
		{
			// scale by time passed
			scale = double(now - start_time) / double(right_eb_angle_time);
			right_eb_angle = old_right_eb_angle + scale * (new_right_eb_angle - old_right_eb_angle);
		}

		eyebrowRight.write(right_eb_angle);
	}

}

/**
 * message order: lift left right
 */
void setEyebrowCB(const nero_msgs::Eyebrows &msg)
{
	old_left_eb_angle = new_left_eb_angle;
	old_right_eb_angle = new_right_eb_angle;

	new_left_eb_angle = msg.left;
	new_right_eb_angle = msg.right;

	left_eb_angle_time = max(1, msg.left_time);
	right_eb_angle_time = max(1, msg.right_time);

        if (new_left_eb_angle > EYEBROW_UPPER_LIMIT)
		new_left_eb_angle = EYEBROW_UPPER_LIMIT;
	else if (new_left_eb_angle < EYEBROW_LOWER_LIMIT)
		new_left_eb_angle = EYEBROW_LOWER_LIMIT;

	if (new_right_eb_angle > EYEBROW_UPPER_LIMIT)
		new_right_eb_angle = EYEBROW_UPPER_LIMIT;
	else if (new_right_eb_angle < EYEBROW_LOWER_LIMIT)
		new_right_eb_angle = EYEBROW_LOWER_LIMIT;  
	

	start_time = millis();
}

//Subscriber to eyebrow commands from eyebrowTopic
ros::Subscriber<nero_msgs::Eyebrows> servo_sub("/eyebrowTopic", &setEyebrowCB);

void setup()
{
	//led
	pinMode(13, OUTPUT);

	nh.initNode();

	//eyebrow init
	old_left_eb_angle = 0;
	old_right_eb_angle = 0;
	new_left_eb_angle = 0;
	new_right_eb_angle = 0;
	left_eb_angle_time = 0;
	right_eb_angle_time = 0;
	start_time = 0;
	nh.subscribe(servo_sub);
	
        eyebrowLeft.attach(EYEBROW_LEFT);
	eyebrowRight.attach(EYEBROW_RIGHT);
	//lift.attach(EYEBROW_LIFT);
	//lift.write(150);
	//delay(500);

	FlexiTimer2::set(150, 1.0/1000, updateEyebrowEvent); // call every 150ms
	FlexiTimer2::start();

	nh.advertise(bat_pub);
	nh.advertise(bat_log_pub);
        memset(batteryValues, 0, sizeof(int)*movingAverageSize);

}



