/*
 * Controller.cpp
 *
 *  Created on: 2011-11-13
 *      Author: wilson
 */

#include "logical_unit/Controller.h"

/// Maps incoming string to a commandValue for use in switch-case in main function
static std::map<std::string, commandValue> stringToValue;

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
void Controller::findObject(u_int8_t object_id)
{
	std_msgs::UInt8 int_msg;
	int_msg.data = object_id;
	mObjectRecognitionPublisher.publish(int_msg);
}

/**
 *	Listens to the position of the head and updates the head's current pose
 */
void Controller::headPoseCB(const geometry_msgs::Pose& msg)
{
	mHeadCurrentPose = msg;
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
	updateRobotPosition();
	/*
	geometry_msgs::PoseStamped msg;
	msg.pose.position.x = std::cos(tf::getYaw());
	msg.pose.position.y = std::sin();
	 */

	geometry_msgs::PoseStamped msg;

	//moveBase(msg);

	//mLock = true;
	//waitForLock();

	//ROS_INFO("Reached Goal");

	//Then, lift the arm just above the table
	moveArm(0.0, 0.0);

	//Aim Kinect to the table
	moveHead(0.0, 0.0);
	//TODO measure the correct angles

	//Start object recognition process
	mLock = true;
	findObject(9387);
	waitForLock();

	mTransformListener.transformPose("arm_frame", mObjectPose, mObjectToArmPose);

	//Move arm to object and grab it
	moveArm(mObjectToArmPose);

	//Tuck in arm while holding object
	moveArm(MAX_ARM_X_VALUE, MIN_ARM_Z_VALUE);

	//Go back to original position
	moveBase(msg);

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
	if(msg.data == FOLLOW_STATE_FINISHED)
		mLock = false;
}

/**
 * Listens to distances to goal
 */
void Controller::baseGoalCB(const std_msgs::Float32& msg)
{
	mDistanceToGoal = msg.data;
}

/**
 * Listens to poses of the found object
 */
void Controller::objectPositionCB(const geometry_msgs::PoseStamped& msg)
{
	mObjectPose = msg;
	mLock = false;
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
	mObjectPoseSubscriber		= mNodeHandle.subscribe("/objectPoseFeedbackTopic", 1, &Controller::objectPositionCB, this);

	//initialise publishers
	mBaseGoalPublisher 			= mNodeHandle.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);
	mArmPositionPublisher 		= mNodeHandle.advertise<geometry_msgs::Pose>("/cmd_arm_position", 1);
	mHeadPositionPublisher		= mNodeHandle.advertise<geometry_msgs::Pose>("/cmd_head_position", 1);
	mObjectRecognitionPublisher = mNodeHandle.advertise<std_msgs::UInt8>("cmd_object_recognition", 1);

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

	char *path = NULL;
	if (argc == 2)
		path = argv[1];

	controller.init();

	ros::spin();
}
