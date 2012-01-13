/*
 * Controller.h
 *
 *  Created on: 2011-11-13
 *      Author: wilson
 */

#ifndef CONTROLLER_H_
#define CONTROLLER_H_

#include <ros/ros.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Float32.h>
#include <logical_unit/PathFollower.h>
#include <audio_processing/speech.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include "yaml-cpp/yaml.h"
#include "fstream"
#include <map>

#define MIN_ARM_Z_VALUE -32.1
#define MAX_ARM_Z_VALUE 17.25
#define MIN_ARM_X_VALUE -20
#define MAX_ARM_X_VALUE 35.95

enum commandValue
{
	NOTHING = -1,
			WAKE_UP,
			JUICE,
			SLEEP,
};

enum Emotions
{
	NEUTRAL = 0,
			HAPPY,
			SAD,
			SURPRISED,
};

class Controller
{
private:
	ros::NodeHandle mNodeHandle;

	ros::Subscriber mSpeechSubscriber;			/// Listens to speech processed by SpeechRecognition node
	ros::Subscriber mObjectPoseSubscriber;		/// Listens to the Pose of the found object
	ros::Subscriber mBaseGoalSubscriber;		/// Listens to path distances to see whether goal is reached
	ros::Subscriber mNavigationStateSubscriber;	///	Listens to the state of the navigation algorithm

	ros::Publisher mBaseGoalPublisher;			/// Publishes goal commands to PathPlanner
	ros::Publisher mArmPositionPublisher;		/// Publishes coordinates to AutonomeArmController
	ros::Publisher mHeadPositionPublisher;		/// Publishes coordinates to AutonomeHeadController
	ros::Publisher mObjectRecognitionPublisher;	/// Publishes ID of the object to recognize

	tf::TransformListener mTransformListener;	/// Fills mOriginalPosition
	tf::StampedTransform mOriginalPosition;		/// Keeps track of the original position of the robot in the map
	//geometry_msgs::Pose mHeadCurrentPose;		/// Keeps track of the current Pose of the head
	geometry_msgs::Pose mGoal;					/// Stores the goal postion

	double mDistanceToGoal;						/// Distance to the goal
	double mDistanceTolerance;					/// Distance tolerance to the goal

	geometry_msgs::PoseStamped mObjectPose;			/// Keeps track of the pose of the found object
	geometry_msgs::PoseStamped mObjectToArmPose;	/// Keeps track of the pose of the found object in arm space

	Emotions mEmotionalState;					/// Keeps track of Nero's current emotion
	std::string mSpeech;						/// Keeps track of what is said by the user
	uint8_t mArousal;							/// Keeps track of the arousal contained in the speech of user
	bool mWakeUp;								/// Keeps track of whether Nero is listening to commands or not
	bool mLock;									/// Checks whether the Controller is blocked or not

public:
	Controller() : mNodeHandle(""){}

	virtual ~Controller(){ mNodeHandle.shutdown();}

	void init();
	void updateRobotPosition();
	void moveArm(double x, double z);
	void moveArm(const geometry_msgs::PoseStamped msg);
	void moveBase(geometry_msgs::PoseStamped &stamped_goal);
	void moveHead(double x, double z);
	void findObject(u_int8_t object_id);

	void getJuice();

	void navigationStateCB(const std_msgs::UInt8& msg);
	void baseGoalCB(const std_msgs::Float32& msg);
	void speechCB(const audio_processing::speech& msg);
	//void headPoseCB(const geometry_msgs::Pose& msg);
	void objectPositionCB(const geometry_msgs::PoseStamped& msg);
	void waitForLock();

	inline void convertQuaternion(geometry_msgs::Quaternion q1, tf::Quaternion q2)
	{
			q1.x = q2.getX();
			q1.y = q2.getY();
			q1.z = q2.getZ();
			q1.w = q2.getW();
	}
};

#endif /* CONTROLLER_H_ */
