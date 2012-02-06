/*
 * PS3Controller.cpp
 *
 *  Created on: Feb 3, 2012
 *      Author: hans
 */

#include "ros/ros.h"
#include "std_msgs/UInt8.h"
#include "head/Emotion.h"
#include "audio_processing/speech.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Bool.h"
#include "sensor_msgs/Joy.h"

#define MAX_LINEAR_SPEED 0.2
#define MAX_ANGULAR_SPEED 0.2

ros::Publisher *emotion_pub;
ros::Publisher *arm_pub;
ros::Publisher *gripper_pub;
ros::Publisher *gripper_state_pub;
ros::Publisher *speech_pub;
ros::Publisher *speed_pub;

enum PS3Key
{
    PS3_NONE = -1,

    PS3_SELECT = 0,
    PS3_LEFT_STICK = 1,
    PS3_RIGHT_STICK = 2,
    PS3_START = 3,

    PS3_UP = 4,
    PS3_RIGHT = 5,
    PS3_DOWN = 6,
    PS3_LEFT = 7,

	PS3_L2 = 8,
	PS3_R2 = 9,
	PS3_L1 = 10,
	PS3_R1 = 11,

	PS3_T = 12,
	PS3_O = 13,
	PS3_X = 14,
    PS3_S = 15,

    PS3_HOME = 16,
};

PS3Key pressedKey;

void joyCB(const sensor_msgs::Joy& msg)
{
	std_msgs::UInt8 uint8msg;
	audio_processing::speech speechmsg;
	std_msgs::Bool boolmsg;
	geometry_msgs::Pose posemsg;
	geometry_msgs::Twist speedmsg;

	if (pressedKey != PS3_NONE && msg.buttons[pressedKey] == 0)
		pressedKey = PS3_NONE;

	if (msg.buttons[PS3_RIGHT_STICK])
	{
		speedmsg.linear.x = msg.axes[1] * MAX_LINEAR_SPEED;
		speedmsg.angular.z = msg.axes[0] * MAX_ANGULAR_SPEED;
		speed_pub->publish(speedmsg);
	}
	else if (pressedKey == PS3_NONE)
	{
		for(size_t i = 0; i < msg.buttons.size(); i++)
		{
			if(msg.buttons[i] == 0)
				continue;

			pressedKey = PS3Key(i);

			switch (i)
			{
			case PS3_X:
				ROS_INFO("Publishing happy state.");
				uint8msg.data = head::Emotion::HAPPY;
				emotion_pub->publish(uint8msg);
				break;
			case PS3_S:
				ROS_INFO("Publishing sad state.");
				uint8msg.data = head::Emotion::SAD;
				emotion_pub->publish(uint8msg);
				break;
			case PS3_O:
				ROS_INFO("Publishing surprised state.");
				uint8msg.data = head::Emotion::SURPRISED;
				emotion_pub->publish(uint8msg);
				break;
			case PS3_T:
				ROS_INFO("Publishing error state.");
				uint8msg.data = head::Emotion::ERROR;
				emotion_pub->publish(uint8msg);
				break;

			case PS3_START:
				ROS_INFO("Publishing wake_up state.");
				speechmsg.arousal = 0;
				speechmsg.command = "wake_up";
				speech_pub->publish(speechmsg);
				break;
			case PS3_SELECT:
				ROS_INFO("Publishing sleep state.");
				speechmsg.arousal = 0;
				speechmsg.command = "sleep";
				speech_pub->publish(speechmsg);
				break;

			case PS3_L1:
				ROS_INFO("Publishing gripper close state.");
				boolmsg.data = false;
				gripper_state_pub->publish(boolmsg);
				break;
			case PS3_L2:
				ROS_INFO("Publishing gripper open state.");
				boolmsg.data = true;
				gripper_state_pub->publish(boolmsg);
				break;

			case PS3_LEFT_STICK:
				ROS_INFO("Publishing gripper (ping) close state.");
				boolmsg.data = false;
				gripper_pub->publish(boolmsg);
				break;

			case PS3_R1:
				ROS_INFO("Publishing arm_up state.");
				posemsg.position.x = 0.0;
				posemsg.position.z = -0.15;
				arm_pub->publish(posemsg);
				break;
			case PS3_R2:
				ROS_INFO("Publishing arm_down state.");
				posemsg.position.x = -1.0;
				posemsg.position.z = -1.0;
				arm_pub->publish(posemsg);
				break;
			}

			break;
		}
	}
}

int main( int argc, char* argv[] )
{
	ros::init(argc, argv, "PS3Controller");
	ros::NodeHandle nh;

	ros::Subscriber joy_sub = nh.subscribe("/joy", 1, &joyCB);
	emotion_pub = new ros::Publisher(nh.advertise<std_msgs::UInt8>("/cmd_emotion", 1));
	gripper_pub = new ros::Publisher(nh.advertise<std_msgs::Bool>("/cmd_gripper", 1));
	gripper_state_pub = new ros::Publisher(nh.advertise<std_msgs::Bool>("/cmd_gripper_state", 1));
	arm_pub = new ros::Publisher(nh.advertise<geometry_msgs::Pose>("/cmd_arm_position", 1));
	speech_pub = new ros::Publisher(nh.advertise<audio_processing::speech>("/processedSpeechTopic", 1));
	speed_pub = new ros::Publisher(nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1));
	pressedKey = PS3_NONE;

	ros::spin();
}
