/*
 * PS3Controller.cpp
 *
 *  Created on: Feb 3, 2012
 *      Author: hans
 */

/**
 * [SELECT] 			- Sleep
 * [START]				- Wake up
 *
 * [CROSS]				- Happy
 * [SQUARE]				- Sad
 * [CIRCLE]				- Surprised
 * [TRIANGLE]			- Angry
 *
 * [UP]					- Give object
 * [LEFT]				- Get cola
 * [RIGHT]				- Get juice
 * [DOWN]				- Stop
 *
 * [L1]					- Close gripper
 * [L2]					- Open gripper
 * [L3]					- Grab with gripper
 *
 * [R1]					- Move arm up
 * [R2]					- Move arm down
 *
 * [R3 + LEFT_STICK]	- Drive
 */

#include "ros/ros.h"
#include "std_msgs/UInt8.h"
#include "nero_msgs/Emotion.h"
#include "nero_msgs/SpeechCommand.h"
#include "nero_msgs/ArmJoint.h"
#include "nero_msgs/PitchYaw.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Bool.h"
#include "sensor_msgs/Joy.h"

#define MAX_LINEAR_SPEED 0.2
#define MAX_ANGULAR_SPEED 0.2

#define MAX_ARM_SPEED 0.2
#define MAX_HEAD_SPEED 0.2

ros::Publisher *emotion_pub;
ros::Publisher *arm_pub;
ros::Publisher *arm_speed_pub;
ros::Publisher *gripper_pub;
ros::Publisher *gripper_state_pub;
ros::Publisher *speech_pub;
ros::Publisher *speed_pub;
ros::Publisher *head_speed_pub;

bool sent_arm_speed = false;
bool sent_base_speed = false;
bool sent_head_speed = false;

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
	nero_msgs::SpeechCommand speechmsg;
	std_msgs::Bool boolmsg;
	geometry_msgs::Pose posemsg;
	geometry_msgs::Twist speedmsg;
	nero_msgs::ArmJoint armspeedmsg;
	nero_msgs::PitchYaw headspeedmsg;

	if (pressedKey != PS3_NONE && msg.buttons[pressedKey] == 0)
		pressedKey = PS3_NONE;

	// arm speeds
	/*if (msg.axes[2] || msg.axes[3])
	{
		armspeedmsg.wrist_joint = msg.axes[2] * MAX_ARM_SPEED;
		armspeedmsg.upper_joint = msg.axes[3] * MAX_ARM_SPEED;
		arm_speed_pub->publish(armspeedmsg);
		sent_arm_speed = true;
	}
	else if (sent_arm_speed)
	{
		sent_arm_speed = false;
		armspeedmsg.wrist_joint = 0.0;
		armspeedmsg.upper_joint = 0.0;
		arm_speed_pub->publish(armspeedmsg);
	}*/

	// base speeds
	if (msg.buttons[PS3_RIGHT_STICK] && (msg.axes[0] || msg.axes[1]))
	{
		speedmsg.linear.x = msg.axes[1] * MAX_LINEAR_SPEED;
		speedmsg.angular.z = msg.axes[0] * MAX_ANGULAR_SPEED;
		speed_pub->publish(speedmsg);
		sent_base_speed = true;
	}
	else if (sent_base_speed)
	{
		sent_base_speed = false;
		speedmsg.linear.x = 0.0;
		speedmsg.angular.z = 0.0;
		speed_pub->publish(speedmsg);
	}

	// head speeds
	/*if (msg.axes[4] || msg.axes[5] || msg.axes[6] || msg.axes[7])
	{
		headspeedmsg.pitch = (msg.axes[4] - msg.axes[6]) * MAX_HEAD_SPEED;
		headspeedmsg.yaw = (msg.axes[5] - msg.axes[7]) * MAX_HEAD_SPEED;
		head_speed_pub->publish(headspeedmsg);
		sent_head_speed = true;
	}
	else if (sent_head_speed)
	{
		sent_head_speed = false;
		headspeedmsg.pitch = 0.0;
		headspeedmsg.yaw = 0.0;
		speed_pub->publish(headspeedmsg);
	}*/

	if (pressedKey == PS3_NONE)
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
				uint8msg.data = nero_msgs::Emotion::HAPPY;
				emotion_pub->publish(uint8msg);
				break;
			case PS3_S:
				ROS_INFO("Publishing sad state.");
				uint8msg.data = nero_msgs::Emotion::SAD;
				emotion_pub->publish(uint8msg);
				break;
			case PS3_O:
				ROS_INFO("Publishing surprised state.");
				uint8msg.data = nero_msgs::Emotion::SURPRISED;
				emotion_pub->publish(uint8msg);
				break;
			case PS3_T:
				ROS_INFO("Publishing error state.");
				uint8msg.data = nero_msgs::Emotion::ERROR;
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
			case PS3_UP:
				ROS_INFO("Publishing give command.");
				speechmsg.arousal = 0;
				speechmsg.command = "give";
				speech_pub->publish(speechmsg);
				break;
			case PS3_LEFT:
				ROS_INFO("Publishing cola command.");
				speechmsg.arousal = 0;
				speechmsg.command = "cola";
				speech_pub->publish(speechmsg);
				break;
			case PS3_RIGHT:
				ROS_INFO("Publishing juice command.");
				speechmsg.arousal = 0;
				speechmsg.command = "juice";
				speech_pub->publish(speechmsg);
				break;
			case PS3_DOWN:
				ROS_INFO("Publishing stop command.");
				speechmsg.arousal = 0;
				speechmsg.command = "stop";
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
	arm_speed_pub = new ros::Publisher(nh.advertise<nero_msgs::ArmJoint>("/arm/cmd_vel", 1));
	speech_pub = new ros::Publisher(nh.advertise<nero_msgs::SpeechCommand>("/processedSpeechTopic", 1));
	speed_pub = new ros::Publisher(nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1));
	head_speed_pub = new ros::Publisher(nh.advertise<nero_msgs::PitchYaw>("/head/cmd_vel", 1));
	pressedKey = PS3_NONE;

	ros::spin();
}
