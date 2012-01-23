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
	mRotateBasePublisher.publish(msg);
}

/**
 * Sends a Pose command to AutonomeArmController, given x and z coordinates
 */
void Controller::moveArm(double x, double z)
{
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
	tf::Quaternion goal_orientation(mOriginalPosition.getRotation().getX(), mOriginalPosition.getRotation().getY(), mOriginalPosition.getRotation().getZ(), mOriginalPosition.getRotation().getW());

	//TODO aanpassen om naar tafel te gaan
	stamped_goal.pose.position.x = std::cos(tf::getYaw(goal_orientation)) + mOriginalPosition.getOrigin().getX();
	stamped_goal.pose.position.y = std::sin(tf::getYaw(goal_orientation)) + mOriginalPosition.getOrigin().getY();

	convertQuaternion(stamped_goal.pose.orientation, goal_orientation);

	stamped_goal.header.stamp = ros::Time::now();

	mBaseGoalPublisher.publish(stamped_goal);
}
/**
 * Sends a Pose command to AutonomeHeadController
 */
void Controller::moveHead(double x, double z)
{
	geometry_msgs::Pose pose_msg;
	pose_msg.position.x = x;
	pose_msg.position.z = z;

	mHeadPositionPublisher.publish(pose_msg);
}

/**
 *	Sends the id of the object to recognize to ObjectRecognition
 */
bool Controller::findObject(int object_id, geometry_msgs::PoseStamped &object_pose)
{
	logical_unit::FindObject find_call;
	find_call.request.objectId = object_id;

	double timer = ros::Time::now().toSec();

	while (ros::Time::now().toSec() - timer < FIND_OBJECT_DURATION)
	{
		if (mFindObjectClient.call(find_call))
		{
			if (find_call.response.result == find_call.response.SUCCESS)
			{
				object_pose = find_call.response.pose;
				return true;
			}
			else
				ROS_ERROR("Something went wrong finding the object.");
		}
		else
			ROS_ERROR("Failed calling finding service.");
	}

	return false;
}

/**
 *	Listens to the speed of the head and releases lock if necessary
 */
void Controller::headSpeedCB(const head::PitchYaw &msg)
{
	if (mLock == LOCK_HEAD && fabs(msg.pitch) < HEAD_FREE_THRESHOLD && fabs(msg.yaw) < HEAD_FREE_THRESHOLD)
		mLock = LOCK_NONE;
}

/**
 *	Listens to the speed of the base and releases lock if necessary
 */
void Controller::baseSpeedCB(const geometry_msgs::Twist &msg)
{
	if (mLock == LOCK_BASE && fabs(msg.linear.x) < BASE_FREE_THRESHOLD && fabs(msg.angular.z) < BASE_FREE_THRESHOLD)
		mLock = LOCK_NONE;
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

/*
 * Method for getting juice
 */
void Controller::getJuice()
{
	//First, rotate head in the right direction

	//Go to the table
	//updateRobotPosition();
	/*
	geometry_msgs::PoseStamped msg;
	msg.pose.position.x = std::cos(tf::getYaw());
	msg.pose.position.y = std::sin();
	 */

	//geometry_msgs::PoseStamped msg;

	//moveBase(msg);

	//mLock = true;
	//waitForLock();

	//ROS_INFO("Reached Goal");

	//Aim Kinect to the table
	mLock = LOCK_HEAD;
	moveHead(VIEW_OBJECTS_ANGLE, 0.0);
	waitForLock();

	//Start object recognition process
	geometry_msgs::PoseStamped objectPose;
	if (findObject(OBJECT_ID, objectPose) == false || objectPose.pose.position.y == 0.0)
	{
		ROS_ERROR("Failed to find object, quitting script.");
		return;
	}

	double yaw = -atan(objectPose.pose.position.x / objectPose.pose.position.y);
	while (yaw > TARGET_YAW_THRESHOLD || fabs(TARGET_DISTANCE - objectPose.pose.position.y) > TARGET_DISTANCE_THRESHOLD)
	{
		if (yaw > TARGET_YAW_THRESHOLD)
		{
			ROS_INFO("Rotating by %lf.", yaw);

			mLock = LOCK_BASE;
			rotateBase(yaw);
			waitForLock();
		}
		else if (fabs(TARGET_DISTANCE - objectPose.pose.position.y) > TARGET_DISTANCE_THRESHOLD)
		{
			ROS_INFO("Moving by %lf.", objectPose.pose.position.y - TARGET_DISTANCE);

			mLock = LOCK_BASE;
			positionBase(objectPose.pose.position.y - TARGET_DISTANCE);
			waitForLock();
		}

		if (findObject(OBJECT_ID, objectPose) == false || objectPose.pose.position.y == 0.0)
		{
			ROS_ERROR("Failed to find object, quitting script.");
			return;
		}

		yaw = -atan(objectPose.pose.position.x / objectPose.pose.position.y);
	}

	//Move arm to object and grab it
	mLock = LOCK_ARM;
	moveArm(objectPose);
	waitForLock();

	mLock = LOCK_BASE;
	positionBase(GRAB_TARGET_DISTANCE);
	waitForLock();

	mLock = LOCK_BASE;
	positionBase(CLEAR_TABLE_DISTANCE);
	waitForLock();

	//Tuck in arm while holding object
	mLock = LOCK_ARM;
	moveArm(MIN_ARM_X_VALUE, objectPose.pose.position.z);
	waitForLock();

	mLock = LOCK_ARM;
	moveArm(MIN_ARM_X_VALUE, MIN_ARM_Z_VALUE);
	waitForLock();

	//Go back to original position
	//moveBase(msg);

	//Deliver the juice
}

/**
 * Listens to user commands and executes them
 */
void Controller::speechCB(const audio_processing::speech& msg)
{
	mSpeech		 = msg.command;
	mArousal 	 = msg.arousal;

	ROS_INFO("String: %s, Value: %d", mSpeech.c_str(), stringToValue[mSpeech]);
	switch(stringToValue[mSpeech])
	{
	case WAKE_UP:
		if(!mWakeUp)
		{
			mWakeUp = true;
			//Start initiating actions to show that the robot has heard the user
			ROS_INFO("Heard user...");
		}
		else
			ROS_INFO("I'm already awake");
		break;

	case JUICE:
		if(mWakeUp)
		{
			//Start initiating actions to get the juice
			ROS_INFO("Getting juice...");
			getJuice();
		}
		else
			ROS_INFO("Don't disturb me in my sleep...");
		break;

	case SLEEP:
		if(mWakeUp)
		{
			//Go to sleep
			mWakeUp = false;
			ROS_INFO("Going to sleep...");
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
//TODO Decide how to score recognized arousal/emotions

/**
 * Listens to navigation state
 */
void Controller::navigationStateCB(const std_msgs::UInt8& msg)
{
	//TODO: Fix this
	//if(msg.data == FOLLOW_STATE_FINISHED)
	//	mLock = NONE;
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
	stringToValue["juice"]   = JUICE;
	stringToValue["sleep"]	 = SLEEP;

	mNodeHandle.param<double>("distance_tolerance", mDistanceTolerance, 0.2);

	//initialise subscribers
	mSpeechSubscriber 			= mNodeHandle.subscribe("/processedSpeechTopic", 1, &Controller::speechCB, this);
	mBaseGoalSubscriber 		= mNodeHandle.subscribe("/path_length", 1, &Controller::baseGoalCB, this);
	mHeadSpeedSubscriber		= mNodeHandle.subscribe("/headSpeedFeedbackTopic", 1, &Controller::headSpeedCB, this);
	mBaseSpeedSubscriber		= mNodeHandle.subscribe("/speedFeedbackTopic", 1, &Controller::baseSpeedCB, this);

	//initialise publishers
	mBaseGoalPublisher 			= mNodeHandle.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);
	mArmPositionPublisher 		= mNodeHandle.advertise<geometry_msgs::Pose>("/cmd_arm_position", 1);
	mHeadPositionPublisher		= mNodeHandle.advertise<geometry_msgs::Pose>("/cmd_head_position", 1);
	mRotateBasePublisher		= mNodeHandle.advertise<std_msgs::Float32>("/cmd_mobile_turn", 1);
	mPositionBasePublisher		= mNodeHandle.advertise<std_msgs::Float32>("/cmd_mobile_position", 1);

	if (waitForServiceClient(&mNodeHandle, "/cmd_find_object"))
		mFindObjectClient		= mNodeHandle.serviceClient<logical_unit::FindObject>("/cmd_find_object", true);

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

	ROS_INFO("Initialised Controller");
}

int main(int argc, char **argv)
{
	// init ros and pathfinder
	ros::init(argc, argv, "controller");
	Controller controller;

	controller.init();

	ros::spin();
}
