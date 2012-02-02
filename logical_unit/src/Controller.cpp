/*
 * Controller.cpp
 *
 *  Created on: 2011-11-13
 *      Author: wilson
 */

#include "logical_unit/Controller.h"

/// Maps incoming string to a commandValue for use in switch-case in main function
static std::map<std::string, commandValue> stringToValue;

/// Publishes stop command to every MotorHandler
ros::Publisher *stopPublisher;

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
	//std_msgs::UInt8 msg;
	//msg.data = emotion;
	//mEmotionPublisher.publish(msg);
	std_msgs::UInt8 msg;
	msg.data = emotion;
	mEmotionPublisher.publish(msg);
}

/**
 * Sends an angle to make the base rotate.
 */
void Controller::rotateBase(float angle)
{
	//Make sure that setMode() for the base is unlocked
	std_msgs::Bool bool_msg;
	bool_msg.data = false;
	stopPublisher->publish(bool_msg);

	std_msgs::Float32 msg;
	msg.data = angle;
	mRotateBasePublisher.publish(msg);
}

void Controller::positionBase(float dist)
{
	//Make sure that setMode() for the base is unlocked
	std_msgs::Bool bool_msg;
	bool_msg.data = false;
	stopPublisher->publish(bool_msg);

	std_msgs::Float32 msg;
	msg.data = dist;
	mPositionBasePublisher.publish(msg);
}

double Controller::positionBaseSpeed(double time, double lin_speed, bool gripperStop)
{
	double start_time = ros::Time::now().toSec();

	geometry_msgs::Twist msg;
	msg.linear.x = lin_speed;

	ros::Rate sleep_rate(50);
	mGripperStop = false;
	while (ros::ok() && ros::Time::now().toSec() - start_time < time && (gripperStop == false || mGripperStop == false))
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
	//Make sure that setMode() for arm is unlocked
	std_msgs::Bool bool_msg;
	bool_msg.data = false;
	stopPublisher->publish(bool_msg);

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
	//Make sure that setMode() for arm is unlocked
	std_msgs::Bool bool_msg;
	bool_msg.data = false;
	stopPublisher->publish(bool_msg);

	mArmPositionPublisher.publish(msg.pose);
}

/**
 * Moves the base to a given goal
 */
void Controller::moveBase(geometry_msgs::Pose &goal)
{
	//Make sure that setMode() for base is unlocked
	std_msgs::Bool bool_msg;
	bool_msg.data = false;
	stopPublisher->publish(bool_msg);

	setFocusFace(false);
	moveHead(0.0, 0.0);

	ROS_INFO("Publishing path goal.");
	geometry_msgs::PoseStamped stamped_goal;
	stamped_goal.pose = goal;
	stamped_goal.header.frame_id = "/map";
	stamped_goal.header.stamp = ros::Time::now();
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
 *	Sends the id of the object to recognise to ObjectRecognition
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
		tf::StampedTransform originalPosition;
		mTransformListener.lookupTransform("/map", "/base_link", ros::Time(0), originalPosition);

		tf::Stamped<tf::Pose> p = tf::Stamped<tf::Pose>(tf::Pose(originalPosition.getRotation(), originalPosition.getOrigin()), ros::Time::now(), originalPosition.frame_id_);
		tf::poseStampedTFToMsg(p, mOriginalPosition);
	}
	catch (tf::TransformException ex)
	{
		//ROS_ERROR("%s",ex.what());
	}
}

void Controller::returnToOriginalPosition()
{
	moveHead(0.0, 0.0);

	mLock = LOCK_PATH;
	moveBase(mOriginalPosition.pose);
	waitForLock();
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

/**
 *	Publishes stop command to MotorHandlers for base and arm.
 *	If an extra MotorHandler is added that needs to be stopped asynchronously, make sure that it listens to the topic /emergencyStop
 */
void stopHandler(int signum)
{
	ROS_INFO("Resetting everything");
	std_msgs::Bool bool_msg;
	bool_msg.data = true;

	stopPublisher->publish(bool_msg);
}

/**
 * Stops with everything and respond
 */
uint8_t Controller::stop()
{
	raise(SIGUSR1);
	signal(SIGUSR1, stopHandler);
	mBusy = false;
	return head::Emotion::SURPRISED;
}
/**
 * Wait after being surprised, before becoming surprised again..
 */
void Controller::waitAfterRespond()
{
	int sleep_rate;
	mNodeHandle.param<int>("node_sleep_rate", sleep_rate, 50);
	ros::Rate sleep(sleep_rate);

	double currentTime = ros::Time::now().toSec();
	while(ros::ok() && ros::Time::now().toSec() - currentTime < 30)
	{
		sleep.sleep();
		ros::spinOnce();
	}

	respondedSurprised = false;
}

/**
 * Respond to the user surprised
 */
uint8_t Controller::respond()
{
	mBusy = true;
	moveHead(HEAD_INIT_X, HEAD_INIT_Z);
	setFocusFace(true);

	mBusy = false;
	respondedSurprised = true;
	return head::Emotion::SURPRISED;
}

/*
 * Method for getting juice
 */
uint8_t Controller::get(int object)
{
	mBusy = true;

	float min_y = 0.f;

	mLock = LOCK_ARM;
	moveArm(MIN_ARM_X_VALUE, MIN_ARM_Z_VALUE);
	waitForLock();

	//Go to the table
	updateRobotPosition();

	moveHead(0.0, 0.0);
	mLock = LOCK_PATH;
	moveBase(mGoal);
	waitForLock();

	ROS_INFO("Reached Goal");

	//Aim Kinect to the table
	setFocusFace(false);
	mLock = LOCK_HEAD;
	moveHead(VIEW_OBJECTS_ANGLE, 0.0);
	waitForLock();

	//Start object recognition process
	geometry_msgs::PoseStamped objectPose;
	double drive_time = 0.0;

	uint8_t attempts = 0;
	for (attempts = 0; attempts < MAX_GRAB_ATTEMPTS; attempts++)
	{
		uint8_t i = 0;
		for (i = 0; i < 3; i++)
		{
			mLock = LOCK_HEAD;
			switch (i)
			{
			case 0: moveHead(VIEW_OBJECTS_ANGLE, 0.0); break;
			case 1: moveHead(MIN_VIEW_ANGLE, 0.0); break;
			case 2: moveHead(MAX_VIEW_ANGLE, 0.0); break;
			}
			waitForLock();

			if (findObject(object, objectPose, min_y) && objectPose.pose.position.y != 0.0)
			{
				// object found with turned head? rotate base
				if (i != 0)
				{
					mLock = LOCK_BASE;
					moveHead(VIEW_OBJECTS_ANGLE, 0.0);
					switch (i)
					{
					case 1: rotateBase(MIN_VIEW_ANGLE); break;
					case 2: rotateBase(MAX_VIEW_ANGLE); break;
					}
					waitForLock();

					if (findObject(object, objectPose, min_y) == false || objectPose.pose.position.y == 0.0)
					{
						ROS_ERROR("Failed to find object, quitting script.");
						returnToOriginalPosition();
						return head::Emotion::SAD;
					}
				}
				break;
			}
		}

		double yaw = -atan(objectPose.pose.position.x / objectPose.pose.position.y);
		while (fabs(yaw) > TARGET_YAW_THRESHOLD || fabs(TABLE_DISTANCE - min_y) > TABLE_DISTANCE_THRESHOLD)
		{
			if (fabs(TABLE_DISTANCE - min_y) > TABLE_DISTANCE_THRESHOLD)
			{
				ROS_INFO("Moving by %lf. Distance: %lf", min_y - TABLE_DISTANCE, min_y);

				mLock = LOCK_BASE;
				positionBase(min_y - TABLE_DISTANCE);
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
				returnToOriginalPosition();
				return head::Emotion::SAD;
			}

			yaw = -atan(objectPose.pose.position.x / objectPose.pose.position.y);
		}

		ROS_INFO("yaw: %lf", yaw);
		ROS_INFO("distance: %lf", fabs(TABLE_DISTANCE - min_y));

		//Move arm to object and grab it
		objectPose.pose.position.x = 0.0; // we are straight ahead of the object, so this should be 0.0 (it is most likely already close to 0.0)
		mLock = LOCK_ARM;
		moveArm(objectPose);
		waitForLock();

		//Be ready to grab object
		setGripper(true);
		usleep(1000000);
		setGripper(false);

		//Move forward to grab the object
		double expected_drive_time = (objectPose.pose.position.y - ARM_LENGTH) / GRAB_TARGET_SPEED;
		drive_time = positionBaseSpeed(expected_drive_time + EXTRA_GRAB_TIME, GRAB_TARGET_SPEED, true);
		if (drive_time >= expected_drive_time + EXTRA_GRAB_TIME)
		{
			ROS_ERROR("Been driving forward too long!");

			// open gripper again
			setGripper(true);

			expressEmotion(head::Emotion::SAD);
			positionBaseSpeed(drive_time, -GRAB_TARGET_SPEED);

			//Move object to body
			mLock = LOCK_ARM;
			moveArm(MIN_ARM_X_VALUE, MIN_ARM_Z_VALUE);
			waitForLock();

			expressEmotion(head::Emotion::NEUTRAL);
		}
		else
			break;
	}

	// no more attempts left, we are sad
	if (attempts >= MAX_GRAB_ATTEMPTS)
	{
		ROS_ERROR("No more grab attempts left.");
		returnToOriginalPosition();
		return head::Emotion::SAD;
	}

	// little hack to allow the gripper to close
	usleep(500000);

	//Lift object
	mLock = LOCK_ARM;
	moveArm(objectPose.pose.position.x, objectPose.pose.position.z + LIFT_OBJECT_DISTANCE);
	waitForLock();

	//Move away from table
	positionBaseSpeed(drive_time, -GRAB_TARGET_SPEED);

	//Move object to body
	mLock = LOCK_ARM;
	moveArm(MIN_ARM_X_VALUE, MIN_ARM_Z_VALUE);
	waitForLock();

	returnToOriginalPosition();

	moveHead(FOCUS_FACE_ANGLE, 0.0);

	//Deliver the juice
	mLock = LOCK_ARM;
	moveArm(DELIVER_ARM_X_VALUE, DELIVER_ARM_Z_VALUE);
	waitForLock();

	/*usleep(5000000);
	setGripper(true);

	moveHead(FOCUS_FACE_ANGLE - 0.1, 0.0);
	usleep(500000);
	moveHead(FOCUS_FACE_ANGLE, 0.0);
	setFocusFace(true);*/

	mBusy = false;

	return head::Emotion::HAPPY;
}

/**
 * Release gripper and deliver the object
 */
uint8_t Controller::release()
{
	mBusy = true;

	//open gripper
	setGripper(true);
	usleep(1000000);

	//Close gripper and move arm back
	std_msgs::Bool bool_msg;
	bool_msg.data = true;

	mGripperClosePublisher.publish(bool_msg);
	moveArm(MIN_ARM_X_VALUE, MIN_ARM_Z_VALUE);

	//Reset arousal and listen to feedback
	mArousal = NEUTRAL_AROUSAL;
	
	double currentTime = ros::Time::now().toSec();
	
	int sleep_rate;
	mNodeHandle.param<int>("node_sleep_rate", sleep_rate, 50);
	ros::Rate sleep(sleep_rate);

	//wait for feedback
	while(ros::ok() && ros::Time::now().toSec() - currentTime < 30 && mArousal == NEUTRAL_AROUSAL)
	{
		sleep.sleep();
		ros::spinOnce();
	}
	
	if(mArousal > NEUTRAL_AROUSAL)
	{
		mBusy = false;
		return head::Emotion::HAPPY;
	}
	else if(mArousal < NEUTRAL_AROUSAL)
	{
		mBusy = false;
		return head::Emotion::SAD;
	}
	else
	{
		setFocusFace(false);
		positionBase(DISTANCE_TO_PERSON);
		mBusy = false;
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

		case EVA:
		case BEER:
			if(!mBusy)
			{
				if(!respondedSurprised)
				{
					ROS_INFO("Responding with surprise");
					expressEmotion(respond());
					waitAfterRespond();
				}
			}
			else
				ROS_INFO("Don't disturb me, I'm busy");
			break;

		case JUICE:
			//Start initiating actions to get the juice
			if(!mBusy)
			{
				ROS_INFO("Getting juice...");
				expressEmotion(get(JUICE_ID));
			}
			else
				ROS_INFO("Don't disturb me, I'm busy");
			break;

		case FANTA:
			//Start initiating actions to get the juice
			if(!mBusy)
			{
				ROS_INFO("Getting fanta...");
				expressEmotion(get(FANTA_ID));
			}
			else
				ROS_INFO("Don't disturb me, I'm busy");
			break;

		case COKE:
		case COLA:
			if(!mBusy)
			{
				//Start initiating actions to get the juice
				ROS_INFO("Getting coke...");
				expressEmotion(get(COLA_ID));
			}
			else
				ROS_INFO("Dont' disturb me, I'm busy");
			break;

		case GIVE:
		case GOT:
			if(!mBusy)
			{
				ROS_INFO("Giving retrived object...");
				expressEmotion(release());
			}
			else
				ROS_INFO("Don't disturb me, I'm busy");
			break;

		case STOP:
			//Stop with whatever Eva is doing and respond
			ROS_INFO("Stopping with everything and respond");
			expressEmotion(stop());
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
	if(mLock == LOCK_PATH && msg.data == 3) // finished
	{
		ROS_INFO("Unlocking path.");
		mLock = LOCK_NONE;
	}
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
void Controller::init(const char *goalPath)
{
	//initialise Nero in sleep mode
	mWakeUp = false;
	mBusy = false;
	respondedSurprised = false;

	//initialise map
	stringToValue[""]		 = NOTHING;
	stringToValue["wake up"] = WAKE_UP;
	stringToValue["eva"]	 = EVA;
	stringToValue["stop"]	 = STOP;
	stringToValue["beer"]	 = BEER;
	stringToValue["cola"]	 = COLA;
	stringToValue["coke"]	 = COKE;
	stringToValue["juice"]   = JUICE;
	stringToValue["give"]	 = GIVE;
	stringToValue["got"]	 = GOT;
	stringToValue["sleep"]	 = SLEEP;
	stringToValue["fanta"]	 = FANTA;

	mNodeHandle.param<double>("distance_tolerance", mDistanceTolerance, 0.2);

	//initialise subscribers
	mSpeechSubscriber 			= mNodeHandle.subscribe("/processedSpeechTopic", 1, &Controller::speechCB, this);
	mBaseGoalSubscriber 		= mNodeHandle.subscribe("/path_length", 1, &Controller::baseGoalCB, this);
	mHeadSpeedSubscriber		= mNodeHandle.subscribe("/headSpeedFeedbackTopic", 1, &Controller::headSpeedCB, this);
	mBaseSpeedSubscriber		= mNodeHandle.subscribe("/speedFeedbackTopic", 1, &Controller::baseSpeedCB, this);
	mArmSpeedSubscriber			= mNodeHandle.subscribe("/armJointSpeedFeedbackTopic", 1, &Controller::armSpeedCB, this);
	mGripperStateSubscriber		= mNodeHandle.subscribe("/gripper_state", 1, &Controller::gripperStateCB, this);
	mPathFollowStateSubscriber	= mNodeHandle.subscribe("/follow_state", 1, &Controller::navigationStateCB, this);

	//initialise publishers
	mBaseGoalPublisher 			= mNodeHandle.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);
	mArmPositionPublisher 		= mNodeHandle.advertise<geometry_msgs::Pose>("/cmd_arm_position", 1);
	mHeadPositionPublisher		= mNodeHandle.advertise<head::PitchYaw>("/cmd_head_position", 1);
	mRotateBasePublisher		= mNodeHandle.advertise<std_msgs::Float32>("/cmd_mobile_turn", 1);
	mPositionBasePublisher		= mNodeHandle.advertise<std_msgs::Float32>("/cmd_mobile_position", 1);
	mGripperCommandPublisher	= mNodeHandle.advertise<std_msgs::Bool>("/cmd_gripper", 1);
	mGripperClosePublisher		= mNodeHandle.advertise<std_msgs::Bool>("/cmd_gripper_state", 1);

	mEmotionPublisher			= mNodeHandle.advertise<std_msgs::UInt8>("/cmd_sound", 1, true);
	mBaseSpeedPublisher			= mNodeHandle.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
	stopPublisher				= new ros::Publisher(mNodeHandle.advertise<std_msgs::Bool>("/emergencyStop", 1));

	if (waitForServiceClient(&mNodeHandle, "/cmd_object_recognition"))
		mFindObjectClient		= mNodeHandle.serviceClient<image_processing::FindObject>("/cmd_object_recognition", true);

	if (waitForServiceClient(&mNodeHandle, "/set_focus_face"))
		mSetFaceFocusClient		= mNodeHandle.serviceClient<image_processing::SetActive>("/set_focus_face", true);

	//Store goal position

	signal(SIGUSR1, stopHandler);

	std::ifstream fin(goalPath);
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

	if (argc !=2)
	{
		ROS_ERROR("Invalid use of Controller. Usage: rosrun logical_unit Controller <path_to_goal_yaml>");
		return 0;
	}

	Controller controller;

	controller.init(argv[1]);
	//controller.expressEmotion(controller.get(JUICE_ID));

	int sleep_rate;
	controller.getNodeHandle()->param<int>("node_sleep_rate", sleep_rate, 50);
	ros::Rate sleep(sleep_rate);

	//controller.expressEmotion(controller.get(COLA_ID));

	while (ros::ok())
	{
		sleep.sleep();
		ros::spinOnce();
	}
}
