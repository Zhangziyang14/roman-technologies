/*
* Roman Technologies 
* - Head rgb control
* - Ping readout
* - 1 publisher: /pingFeedbackTopic
* - 2 subscribers: /rgbTopic /pingActivateTopic
* Author: Ingmar Jager
*/

#include <ros.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Bool.h>
#include <head/RGB.h>
#include <FlexiTimer2.h>

int r, g, b;				// current RGB values
int old_r, old_g, old_b;	// previous set RGB values (used for smooth transitions between RGB commands)
int min_r, min_g, min_b;	// lower RGB limit
int max_r, max_g, max_b;	// upper RGB limit
unsigned long start_time;			// time at which breathing started
unsigned long breath_time;			// time a breath cycle takes
boolean transition_rgb;				// determines whether we are transitioning from old_r/g/b to min_r/g/b

#define TRANSITION_TIME 500

//Pin definitions
#define RED_PIN 6
#define GREEN_PIN 5
#define BLUE_PIN 9

#define PING_PIN 8

//ping properties
boolean pingActivated;
std_msgs::UInt16 ping_msg;

//ROS
ros::NodeHandle nh;
ros::Publisher ping_pub("/pingFeedbackTopic", &ping_msg);

////////////////////////////////////////////////////////////////////////////////////////
//                                   RGB Section
////////////////////////////////////////////////////////////////////////////////////////

/**
* Called every timer interrupt
* Changes colors fluently
* Colors fade up and down
*/
void updateRGBEvent()
{
	unsigned long now = millis();

	// transitioning from one RGB command to another
	if (transition_rgb)
	{
		if (now - start_time > TRANSITION_TIME)
		{
			// we no longer need to do the transition
			transition_rgb = false;
			start_time = now;
		}
		else
		{
			double scale = 0.5 * sin(((now - start_time) / double(TRANSITION_TIME)) * 3.14 - 1.57) + 0.5;

			// transition from old_rgb to min_rgb
			r = old_r + scale * (min_r - old_r);
			g = old_g + scale * (min_g - old_g);
			b = old_b + scale * (min_b - old_b);
		}
	}
	else // breathing
	{
		r = min_r;
		g = min_g;
		b = min_b;

		if (breath_time)
		{
			double scale = 0.5 * sin(((now - start_time) / double(breath_time)) * 6.28 - 1.57) + 0.5;

			r += scale * (max_r - min_r);
			g += scale * (max_g - min_g);
			b += scale * (max_b - min_b);
		}
	}

	analogWrite(RED_PIN, r);
	analogWrite(GREEN_PIN, g);
	analogWrite(BLUE_PIN, b);
}

/**
* sets designated color targets received through rgbTopic
*/
void setRGBCB(const head::RGB &msg)
{
	// store current RGB for smooth transition
	old_r = r;
	old_g = g;
	old_b = b;
	transition_rgb = true;

	min_r = msg.min_r;
	min_g = msg.min_g;
	min_b = msg.min_b;
	max_r = msg.max_r;
	max_g = msg.max_g;
	max_b = msg.max_b;
	breath_time = msg.breath_time;

	start_time = millis();
}

ros::Subscriber<head::RGB> rgb_sub("/rgbTopic", &setRGBCB);
////////////////////////////////////////////////////////////////////////////////////////
//                                    ping Section
////////////////////////////////////////////////////////////////////////////////////////


/**
* Start sending out an ultrasonic pulse
*/
void doPulse()
{
	pinMode(PING_PIN, OUTPUT);
	digitalWrite(PING_PIN, LOW);
	delayMicroseconds(2);
	digitalWrite(PING_PIN, HIGH);
	delayMicroseconds(5);
	digitalWrite(PING_PIN, LOW); 
}  

/**
* Reading is done from same pin as the output pin, so change pinMode. a HIGH
* pulse whose duration is the time (in microseconds) from the sending
* of the ping to the reception of its echo off an object.
*/
long readPulse()
{   
	pinMode(PING_PIN, INPUT);
	return pulseIn(PING_PIN, HIGH); 
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
	if(pingActivated)
	{
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
void activatePingCB(const std_msgs::Bool& msg)
{
	pingActivated = msg.data;
}

ros::Subscriber<std_msgs::Bool> ping_sub("/pingActivateTopic", &activatePingCB);

////////////////////////////////////////////////////////////////////////////////////////
//                                       setup & loop Section
////////////////////////////////////////////////////////////////////////////////////////

void setup()
{
	nh.initNode();

	//rgb init
	nh.subscribe(rgb_sub);
	pinMode(RED_PIN, OUTPUT);
	pinMode(GREEN_PIN, OUTPUT);
	pinMode(BLUE_PIN, OUTPUT);
	r = 0;
	g = 0;
	b = 0;
	old_r = 0;
	old_g = 0;
	old_b = 0;
	min_r = 0;
	min_g = 0;
	min_b = 0;
	max_r = 0;
	max_g = 0;
	max_b = 0;
	breath_time = 0;
	start_time = 0;
	transition_rgb = false;

	//ping init
	pingActivated = false;
	nh.advertise(ping_pub);
	nh.subscribe(ping_sub);

	FlexiTimer2::set(15, 1.0/1000, updateRGBEvent); // call every 15ms
	FlexiTimer2::start();
}

void loop()
{
	ping();
	nh.spinOnce();
}

