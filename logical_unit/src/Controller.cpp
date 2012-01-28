/*
 * Controller.cpp
 *
 *  Created on: 2011-11-13
 *      Author: wilson
 */

#include "logical_unit/Controller.h"

/// Maps incoming string to a commandValue for use in switch-case in main function
static std::map<std::string, commandValue> stringToValue;

bool waitForServiceClient(ros::NodeHandle *nodeHandle, const char *serviceName)
{
	//wait for client
	while (!ros::service::waitForService(serviceName, ros::Duration(2.0)) && nodeHandle->ok())
	{
		ROS_INFO("Waiting for object detection service to come up");
	}
	if (nodeHandle->ok() == false)
	{
		ROS_ERROR("Error waiting for %s", serviceName);
		return false;
	}
	return true;
}

void Controller::gripperStateCB(const std_msgs::UInt8 &msg)
{
	if (msg.data == GS_CLOSED)
	{
		positionBase(0.f);
		mGripperStop = true;
	}
}

void Controller::expressEmotion(uint8_t emotion)
{
	std_msgs::UInt8 msg;
	msg.data = emotion;
	mEmotionPublisher.publish(msg);
}

/**
 * Sends an angle to make the base rotate.
 */
void Controller::rotateBase(float angle)
{
	std_msgs::Float32 msg;
	msg.data = angle;
	mRotateBasePublisher.publish(msg);
}

void Controller::positionBase(float dist)
{
	std_msgs::Float32 msg;
	msg.data = dist;
	mPositionBasePublisher.publish(msg);
}

double Controller::positionBaseSpeed(double time, double lin_speed)
{
	double start_time = ros::Time::now().toSec();

	geometry_msgs::Twist msg;
	msg.linear.x = lin_speed;

	ros::Rate sleep_rate(50);
	mGripperStop = false;
	while (ros::ok() && ros::Time::now().toSec() - start_time < time && mGripperStop == false)
	{
		mBaseSpeedPublisher.publish(msg);
		sleep_rate.sleep();
		ros::spinOnce();
	}

	msg.linear.x = 0.0;
	mBaseSpeedPublisher.publish(msg);
	return ros::Time::now().toSec() - start_time;
}

void Controller::setGripper(bool open)
{
	std_msgs::Bool bool_msg;
	bool_msg.data = open;

	ROS_INFO("%s gripper", open ? "Opening" : "Closing");
	mGripperCommandPublisher.publish(bool_msg);
}

/**
 * Sends a Pose command to AutonomeArmController, given x and z coordinates
 */
void Controller::moveArm(double x, double z)
{
	ROS_INFO("Moving arm to %lf, %lf", x, z);
	geometry_msgs::Pose pose_msg;
	pose_msg.position.x = x;
	pose_msg.position.z = z;

	mArmPositionPublisher.publish(pose_msg);
}

/**
 * Sends a Pose command to AutonomeArmController, given a Posestamped
 */
void Controller::moveArm(const geometry_msgs::PoseStamped msg)
{
	mArmPositionPublisher.publish(msg.pose);
}

void Controller::moveBase(geometry_msgs::PoseStamped &stamped_goal)
{
	ROS_INFO("Publishing path goal.");
	mBaseGoalPublisher.publish(stamped_goal);
}
/**
 * Sends a Pose command to AutonomeHeadController
 */
void Controller::moveHead(double x, double z)
{
	ROS_INFO("Moving head to pitch: %lf, yaw: %lf", x, z);
	head::PitchYaw head_msg;
	head_msg.pitch = x;
	head_msg.yaw = z;

	mHeadPositionPublisher.publish(head_msg);
}

/**
 *	Sends the id of the object to recognize to ObjectRecognition
 */
bool Controller::findObject(int object_id, geometry_msgs::PoseStamped &object_pose, float &min_y)
{
	image_processing::FindObject find_call;
	find_call.request.objectId = object_id;

	double timer = ros::Time::now().toSec();

	ROS_INFO("Starting search for object with id: %d", object_id);
	while (ros::Time::now().toSec() - timer < FIND_OBJECT_DURATION)
	{
		if (mFindObjectClient.call(find_call))
		{
			if (find_call.response.result == find_call.response.SUCCESS)
			{
				object_pose = find_call.response.pose;
				object_pose.pose.position.z = find_call.response.table_z + GRAB_OBJECT_Z_OFFSET;
				min_y = find_call.response.min_y;
				return true;
			}
			else
				ROS_ERROR("Something went wrong finding the object.");
		}
		else
			ROS_ERROR("Failed calling finding service.");
	}

	ROS_INFO("Finding object failed.");
	return false;
}

void Controller::setFocusFace(bool active)
{
	image_processing::SetActive active_call;
	active_call.request.active = active;
	mSetFaceFocusClient.call(active_call);

	ROS_INFO("%s focus face.", active ? "Activating" : "De-activating");
}

/**
 *	Listens to the speed of the head and releases lock if necessary
 */
void Controller::headSpeedCB(const head::PitchYaw &msg)
{
	if (mLock == LOCK_HEAD && fabs(msg.pitch) < HEAD_FREE_THRESHOLD && fabs(msg.yaw) < HEAD_FREE_THRESHOLD)
	{
		mLock = LOCK_NONE;
		ROS_INFO("UNLOCKING HEAD");
	}
}

/**
 *	Listens to the speed of the base and releases lock if necessary
 */
void Controller::baseSpeedCB(const geometry_msgs::Twist &msg)
{
	if (mLock == LOCK_BASE && fabs(msg.linear.x) < BASE_FREE_THRESHOLD && fabs(msg.angular.z) < BASE_FREE_THRESHOLD)
	{
		mLock = LOCK_NONE;
		ROS_INFO("UNLOCKING BASE");
	}
}

/**
 *	Listens to the speed of the base and releases lock if necessary
 */
void Controller::armSpeedCB(const arm::armJointPos &msg)
{
	if (mLock == LOCK_ARM && fabs(msg.upper_joint) < ARM_FREE_THRESHOLD && fabs(msg.wrist_joint) < ARM_FREE_THRESHOLD)
	{
		mLock = LOCK_NONE;
		ROS_INFO("UNLOCKING ARM");
	}
}

/**
 *	Updates robot position on map
 */
void Controller::updateRobotPosition()
{
	try
	{
		mTransformListener.lookupTransform("/map", "/base_link", ros::Time(0), mOriginalPosition);
	}
	catch (tf::TransformException ex)
	{
		//ROS_ERROR("%s",ex.what());
	}
}

/**
 * Blocks Controller until mLock is free
 */
void Controller::waitForLock()
{
	usleep(LOCK_STARTUP_TIME * 1000 * 1000);
	while(ros::ok() && mLock)
	{
		ros::spinOnce();
		usleep(200000);
	}
}

/**
 * Actions to do when waking up
 */
uint8_t Controller::wakeUp()
{
	mWakeUp = true;
	moveHead(HEAD_INIT_X, HEAD_INIT_Z);
	return head::Emotion::NEUTRAL;
}

/**
 * Actions to do when going to sleep
 */
uint8_t Controller::sleep()
{
	mWakeUp = false;
	moveHead(HEAD_SLEEP_X, HEAD_SLEEP_Z);
	return head::Emotion::SLEEP;
}

/*
 * Method for getting juice
 */
uint8_t Controller::get(int object)
{
	float min_y = 0.f;

	mLock = LOCK_ARM;
	moveArm(MIN_ARM_X_VALUE, MIN_ARM_Z_VALUE);
	waitForLock();

	//Go to the table
	updateRobotPosition();

	geometry_msgs::PoseStamped msg;
	msg.pose = mGoal;
	msg.header.stamp = ros::Time::now();
	msg.header.frame_id = "/map";
	moveBase(msg);
	mLock = LOCK_PATH;
	waitForLock();

	ROS_INFO("Reached Goal");

	//Aim Kinect to the table
	setFocusFace(false);
	mLock = LOCK_HEAD;
	moveHead(VIEW_OBJECTS_ANGLE, 0.0);
	waitForLock();

	//Start object recognition process
	geometry_msgs::PoseStamped objectPose;
	if (findObject(object, objectPose, min_y) == false || objectPose.pose.position.y == 0.0)
	{
		ROS_ERROR("Failed to find object, quitting script.");
		return head::Emotion::SAD;
	}

	double yaw = -atan(objectPose.pose.position.x / objectPose.pose.position.y);
	while (fabs(yaw) > TARGET_YAW_THRESHOLD || fabs(TARGET_DISTANCE - objectPose.pose.position.y) > TARGET_DISTANCE_THRESHOLD)
	{
		if (fabs(TARGET_DISTANCE - objectPose.pose.position.y) > TARGET_DISTANCE_THRESHOLD)
		{
			ROS_INFO("Moving by %lf. Distance: %lf", objectPose.pose.position.y - TARGET_DISTANCE, objectPose.pose.position.y);

			mLock = LOCK_BASE;
			positionBase(objectPose.pose.position.y - TARGET_DISTANCE);
			waitForLock();
		}
		else
		{
			ROS_INFO("Rotating by %lf.", yaw);

			mLock = LOCK_BASE;
			rotateBase(yaw);
			waitForLock();
		}

		if (findObject(object, objectPose, min_y) == false || objectPose.pose.position.y == 0.0)
		{
			ROS_ERROR("Failed to find object, quitting script.");
			return head::Emotion::SAD;
		}

		yaw = -atan(objectPose.pose.position.x / objectPose.pose.position.y);
	}

	ROS_INFO("yaw: %lf", yaw);
	ROS_INFO("distance: %lf", fabs(TARGET_DISTANCE - objectPose.pose.position.y));

	//Move arm to object and grab it
	objectPose.pose.position.x = 0.0;
	mLock = LOCK_ARM;
	moveArm(MIN_ARM_X_VALUE, objectPose.pose.position.z);
	waitForLock();
	mLock = LOCK_ARM;
	moveArm(objectPose);
	waitForLock();

	//Be ready to grab object
	setGripper(true);
	usleep(1000000);
	setGripper(false);

	//Move forward to grab the object
	/*mLock = LOCK_BASE;
	positionBase(GRAB_TARGET_DISTANCE);
	waitForLock();*/
	double drive_time = positionBaseSpeed(GRAB_TARGET_TIME, GRAB_TARGET_SPEED);
	if (drive_time >= GRAB_TARGET_TIME)
	{
		ROS_ERROR("Been driving forward too long!");
		return head::Emotion::SAD;
	}

	usleep(500000);

	//Lift object
	mLock = LOCK_ARM;
	moveArm(objectPose.pose.position.x, objectPose.pose.position.z + LIFT_OBJECT_DISTANCE);
	waitForLock();

	ROS_INFO("mLock: %d", mLock);

	//Move away from table
	/*mLock = LOCK_BASE;
	positionBase(CLEAR_TABLE_DISTANCE);
	waitForLock();*/
	positionBaseSpeed(drive_time, -GRAB_TARGET_SPEED);

	//Move object to body
	mLock = LOCK_ARM;
	moveArm(MIN_ARM_X_VALUE, MIN_ARM_Z_VALUE);
	waitForLock();

	//TODO: Move back to the owner

	moveHead(FOCUS_FACE_ANGLE, 0.0);
	setFocusFace(true);

	//Go back to original position
	msg.pose.position.x = mOriginalPosition.getOrigin().getX();
	msg.pose.position.y = mOriginalPosition.getOrigin().getY();
	msg.pose.position.z = mOriginalPosition.getOrigin().getZ();
	msg.pose.orientation.x = mOriginalPosition.getRotation().getX();
	msg.pose.orientation.y = mOriginalPosition.getRotation().getY();
	msg.pose.orientation.z = mOriginalPosition.getRotation().getZ();
	msg.pose.orientation.w = mOriginalPosition.getRotation().getW();
	msg.header.stamp = ros::Time::now();
	moveBase(msg);

	//Deliver the juice

	return head::Emotion::HAPPY;
}

/**
 * Release gripper and deliver the object
 */
uint8_t Controller::release()
{
	setGripper(true);

	//Reset arousal and listen to feedback
	mArousal = NEUTRAL_AROUSAL;

	if(mArousal > NEUTRAL_AROUSAL)
		return head::Emotion::HAPPY;
	else if(mArousal < NEUTRAL_AROUSAL)
		return head::Emotion::SAD;
	else
	{
		//Move away from client
		return head::Emotion::NEUTRAL;
	}
}
/**
 * Listens to user commands and executes them
 */
void Controller::speechCB(const audio_processing::speech& msg)
{
	mSpeech		 = msg.command;
	mArousal 	 = msg.arousal;

	ROS_INFO("String: %s, Value: %d", mSpeech.c_str(), stringToValue[mSpeech]);
	ROS_INFO("Arousal: %d", mArousal);
	if (stringToValue[mSpeech] != WAKE_UP && mWakeUp == false)
		ROS_INFO("Don't disturb me in my sleep...");
	else
	{
		switch(stringToValue[mSpeech])
		{
		case WAKE_UP:
			if(!mWakeUp)
			{
				//Start initiating actions to show that the robot has heard the user
				ROS_INFO("Heard user...");
				expressEmotion(wakeUp());
				setFocusFace(true);
			}
			else
				ROS_INFO("I'm already awake");
			break;

		case JUICE:
			//Start initiating actions to get the juice
			ROS_INFO("Getting juice...");
			expressEmotion(get(JUICE_ID));
			break;

		case COKE:
		case COLA:
			//Start initiating actions to get the juice
			ROS_INFO("Getting coke...");
			expressEmotion(get(COLA_ID));
			break;

		case RELEASE:
			//Start initiating actions to get the juice
			ROS_INFO("Getting coke...");
			expressEmotion(release());
			break;

		case SLEEP:
			if(mWakeUp)
			{
				//Go to sleep
				ROS_INFO("Going to sleep...");
				expressEmotion(sleep());
				setFocusFace(false);
			}
			else
				ROS_INFO("I'm already sleeping...");
			break;

		case NOTHING:
			ROS_INFO("No commands");
		default:
			break;

		}
	}
}

/**
 * Listens to navigation state
 */
void Controller::navigationStateCB(const std_msgs::UInt8& msg)
{
	ROS_INFO("Received navigation update: %d", msg.data);
	if(mLock == LOCK_PATH && msg.data == 3) // finished
		mLock = LOCK_NONE;
}

/**
 * Listens to distances to goal
 */
void Controller::baseGoalCB(const std_msgs::Float32& msg)
{
	mDistanceToGoal = msg.data;
}

/**
 * Initialise Controller
 */
void Controller::init()
{
	//initialise Nero in sleep mode
	mWakeUp = false;

	//initialise map
	stringToValue[""]		 = NOTHING;
	stringToValue["wake up"] = WAKE_UP;
	stringToValue["eva"]	 = EVA;
	stringToValue["cola"]	 = COLA;
	stringToValue["coke"]	 = COKE;
	stringToValue["juice"]   = JUICE;
	stringToValue["release"] = RELEASE;
	stringToValue["sleep"]	 = SLEEP;

	mNodeHandle.param<double>("distance_tolerance", mDistanceTolerance, 0.2);

	//initialise subscribers
	mSpeechSubscriber 			= mNodeHandle.subscribe("/processedSpeechTopic", 1, &Controller::speechCB, this);
	mBaseGoalSubscriber 		= mNodeHandle.subscribe("/path_length", 1, &Controller::baseGoalCB, this);
	mHeadSpeedSubscriber		= mNodeHandle.subscribe("/headSpeedFeedbackTopic", 1, &Controller::headSpeedCB, this);
	mBaseSpeedSubscriber		= mNodeHandle.subscribe("/speedFeedbackTopic", 1, &Controller::baseSpeedCB, this);
	mArmSpeedSubscriber			= mNodeHandle.subscribe("/armJointSpeedFeedbackTopic", 1, &Controller::armSpeedCB, this);
	mGripperStateSubscriber		= mNodeHandle.subscribe("/gripper_state", 1, &Controller::gripperStateCB, this);

	//initialise publishers
	mBaseGoalPublisher 			= mNodeHandle.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);
	mArmPositionPublisher 		= mNodeHandle.advertise<geometry_msgs::Pose>("/cmd_arm_position", 1);
	mHeadPositionPublisher		= mNodeHandle.advertise<head::PitchYaw>("/cmd_head_position", 1);
	mRotateBasePublisher		= mNodeHandle.advertise<std_msgs::Float32>("/cmd_mobile_turn", 1);
	mPositionBasePublisher		= mNodeHandle.advertise<std_msgs::Float32>("/cmd_mobile_position", 1);
	mGripperCommandPublisher	= mNodeHandle.advertise<std_msgs::Bool>("/cmd_gripper", 1);
	mEmotionPublisher			= mNodeHandle.advertise<std_msgs::UInt8>("/cmd_emotion", 1, true);
	mBaseSpeedPublisher			= mNodeHandle.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

	/*if (waitForServiceClient(&mNodeHandle, "/cmd_object_recognition"))
		mFindObjectClient		= mNodeHandle.serviceClient<image_processing::FindObject>("/cmd_object_recognition", true);

	if (waitForServiceClient(&mNodeHandle, "/set_focus_face"))
		mSetFaceFocusClient		= mNodeHandle.serviceClient<image_processing::SetActive>("/set_focus_face", true);*/

	//Store goal position

	std::ifstream fin("config/goal.yaml");
	if (fin.fail())
	{
		ROS_ERROR("Failed to open YAML file.");
		return;
	}

	YAML::Parser parser(fin);
	YAML::Node doc;
	parser.GetNextDocument(doc);

	float yaw;
	try
	{
		doc["goal"][0] >> mGoal.position.x;
		doc["goal"][1] >> mGoal.position.y;
		doc["goal"][2] >> yaw;

		convertQuaternion(mGoal.orientation, tf::createQuaternionFromYaw(yaw));

	} catch (YAML::InvalidScalar)
	{
		ROS_ERROR("No goal found in yaml file");
		return;
	}
	ROS_INFO("x: %f, y: %f, yaw: %f", mGoal.position.x, mGoal.position.y, yaw);

	usleep(1000000);
	expressEmotion(head::Emotion::SLEEP);
	ROS_INFO("Initialised Controller");
}

int main(int argc, char **argv)
{
	// init ros and pathfinder
	ros::init(argc, argv, "controller");
	Controller controller;

	controller.init();
	//controller.expressEmotion(controller.get(JUICE_ID));

	int sleep_rate;
	controller.getNodeHandle()->param<int>("node_sleep_rate", sleep_rate, 50);
	ros::Rate sleep(sleep_rate);

	while (ros::ok())
	{
		sleep.sleep();
		ros::spinOnce();
	}
}
