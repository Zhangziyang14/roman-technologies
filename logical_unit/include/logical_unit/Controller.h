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
#include <audio_processing/speech.h>
#include <std_msgs/Bool.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include "yaml-cpp/yaml.h"
#include "fstream"
#include <map>
#include "image_processing/FindObject.h"
#include "head/PitchYaw.h"
#include "arm/armJointPos.h"
#include "head/Emotion.h"
#include "image_processing/SetActive.h"

#define MIN_ARM_Z_VALUE (-0.321)
#define MAX_ARM_Z_VALUE (0.1725)
#define MAX_ARM_X_VALUE (0.20)
#define MIN_ARM_X_VALUE (-0.3595)

#define HEAD_INIT_X  0.0
#define HEAD_INIT_Z  0.0

#define HEAD_SLEEP_X  0.0
#define HEAD_SLEEP_Z  -0.6

#define VIEW_OBJECTS_ANGLE	0.4

#define TARGET_YAW_THRESHOLD 0.05
#define TARGET_DISTANCE_THRESHOLD 0.1
#define TARGET_DISTANCE 1.05
#define GRAB_TARGET_DISTANCE 0.3
#define CLEAR_TABLE_DISTANCE 0.5

#define LOCK_STARTUP_TIME	 0.5
#define FIND_OBJECT_DURATION 5.0

#define OBJECT_ID 18904

#define HEAD_FREE_THRESHOLD 0.001
#define BASE_FREE_THRESHOLD 0.001
#define ARM_FREE_THRESHOLD 0.001

#define GRAB_OBJECT_Z_OFFSET 0.05

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

enum Locks
{
	LOCK_NONE = 0,
	LOCK_PATH,
	LOCK_BASE,
	LOCK_HEAD,
	LOCK_ARM,
	LOCK_GRIPPER,
};

class Controller
{
private:
	ros::NodeHandle mNodeHandle;

	ros::Subscriber mSpeechSubscriber;				/// Listens to speech processed by SpeechRecognition node
	ros::Subscriber mBaseGoalSubscriber;			/// Listens to path distances to see whether goal is reached
	ros::Subscriber mNavigationStateSubscriber;		///	Listens to the state of the navigation algorithm
	ros::Subscriber mHeadSpeedSubscriber;			/// Listens to head motor speeds
	ros::Subscriber mBaseSpeedSubscriber;			/// Listens to base motor speeds
	ros::Subscriber mArmSpeedSubscriber;			/// Listens to arm motor speeds

	ros::Publisher mGripperCommandPublisher;		/// Publishes commands to gripper Ultrasone controller
	ros::Publisher mBaseGoalPublisher;				/// Publishes goal commands to PathPlanner
	ros::Publisher mArmPositionPublisher;			/// Publishes coordinates to AutonomeArmController
	ros::Publisher mHeadPositionPublisher;			/// Publishes coordinates to AutonomeHeadController
	ros::Publisher mRotateBasePublisher;			/// Publishes rotation angle for base
	ros::Publisher mPositionBasePublisher;			/// Publishes rotation angle for base
	ros::Publisher mEmotionPublisher;				/// Publishes emotions for the head

	ros::ServiceClient mFindObjectClient;			/// Service client for rotating the base
	ros::ServiceClient mSetFaceFocusClient;			/// Service client for activating face detection

	tf::TransformListener mTransformListener;		/// Fills mOriginalPosition
	tf::StampedTransform mOriginalPosition;			/// Keeps track of the original position of the robot in the map
	geometry_msgs::Pose mGoal;						/// Stores the goal postion

	double mDistanceToGoal;							/// Distance to the goal
	double mDistanceTolerance;						/// Distance tolerance to the goal

	Emotions mEmotionalState;						/// Keeps track of Nero's current emotion
	std::string mSpeech;							/// Keeps track of what is said by the user
	uint8_t mArousal;								/// Keeps track of the arousal contained in the speech of user
	bool mWakeUp;									/// Keeps track of whether Nero is listening to commands or not
	Locks mLock;									/// Checks whether the Controller is blocked or not and keeps track of which node it is being blocked by

public:
	Controller() : mNodeHandle(""){}

	virtual ~Controller(){ mNodeHandle.shutdown();}

	void init();
	void updateRobotPosition();
	void moveArm(double x, double z);
	void moveArm(const geometry_msgs::PoseStamped msg);
	void moveBase(geometry_msgs::PoseStamped &stamped_goal);
	void moveHead(double x, double z);
	bool findObject(int object_id, geometry_msgs::PoseStamped &object_pose);
	void setFocusFace(bool active);
	void rotateBase(float angle);
	void positionBase(float dist);
	void setGripper(bool open);
	void expressEmotion(uint8_t emotion);

	uint8_t getJuice();
	uint8_t wakeUp();
	uint8_t sleep();

	void navigationStateCB(const std_msgs::UInt8& msg);
	void baseGoalCB(const std_msgs::Float32& msg);
	void speechCB(const audio_processing::speech& msg);
	void headSpeedCB(const head::PitchYaw &msg);
	void baseSpeedCB(const geometry_msgs::Twist &msg);
	void armSpeedCB(const arm::armJointPos &msg);
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
