/*
 * GrabDemo.cpp
 *
 *  Created on: Jun 15, 2012
 *      Author: tux
 */


#include "logical_unit/GrabDemo.h"

GrabDemo::GrabDemo() :
	mTarget(TARGET_NONE)
{
	mHeadPub			= mNodeHandle.advertise<nero_msgs::PitchYaw>("/cmd_head_position", 1);
	mEmotionPub			= mNodeHandle.advertise<nero_msgs::Emotion>("/cmd_emotion", 1);

	mCloudServerClient	= mNodeHandle.serviceClient<nero_msgs::GetCloud>("/KinectServer/CloudServer", true);
	mForceDepthClient	= mNodeHandle.serviceClient<nero_msgs::SetActive>("/KinectServer/ForceDepth", true);
	mSegmentTableClient	= mNodeHandle.serviceClient<nero_msgs::SegmentTable>("/KinectServer/segment_table", true);
	mSegmentTableClient	= mNodeHandle.serviceClient<nero_msgs::SetActive>("/set_focus_face", true);

	mCommandSub			= mNodeHandle.subscribe("/GrabDemo/grab_id", 1, &GrabDemo::grabCb, this);
	mHeadPosSub			= mNodeHandle.subscribe("/headPositionFeedbackTopic", 1, &GrabDemo::headPosCb, this);
	mBasePosSub			= mNodeHandle.subscribe("/MobileBase/position", 1, &GrabDemo::basePosCb, this);

	mHeadPos.pitch = 0.0;
	mHeadPos.yaw = 0.0;
	mBasePos.left = 0.0;
	mBasePos.right = 0.0;
}

bool GrabDemo::createTableDistance()
{
	while (true)
	{
		sensor_msgs::PointCloud2 pc;
		if (getCloud(pc) == false)
		{
			ROS_ERROR("[GrabDemo] Failed to get the PointCloud.");
			return false;
		}

		nero_msgs::Table table;
		if (segmentTable(table, pc) == false)
		{
			ROS_ERROR("[GrabDemo] Failed to segment the table.");
			return false;
		}

		if (waitForBasePos(table.y_min - TABLE_DISTANCE) == false)
			return false;
	}

	return true;
}

bool GrabDemo::waitForBasePos(double distance)
{
	ros::Rate rate(10);
	double start = ros::Time::now().toSec();

	nero_msgs::MotorPosition startPos = mBasePos;
	setBase(distance);

	while (ros::ok() && ros::Time::now().toSec() - start < MAX_WAIT)
	{
		if (fabs(fabs(mBasePos.left - startPos.left) - distance) < BASE_FREE_THRESHOLD &&
			fabs(fabs(mBasePos.right - startPos.right) - distance) < HEAD_FREE_THRESHOLD)
			break;

		ros::spinOnce();
		rate.sleep();
	}

	if (ros::Time::now().toSec() - start >= MAX_WAIT)
	{
		ROS_ERROR("[GrabDemo] Failed to set the base with distance %lf", distance);
		return false;
	}

	return true;
}

bool GrabDemo::waitForHeadPos(double pitch, double yaw)
{
	ros::Rate rate(10);
	double start = ros::Time::now().toSec();

	while (ros::ok() && ros::Time::now().toSec() - start < MAX_WAIT)
	{
		if (fabs(mHeadPos.pitch - pitch) < HEAD_FREE_THRESHOLD && fabs(mHeadPos.yaw - yaw) < HEAD_FREE_THRESHOLD)
			break;

		setHead(pitch, yaw);
		ros::spinOnce();
		rate.sleep();
	}

	if (ros::Time::now().toSec() - start >= MAX_WAIT)
	{
		ROS_ERROR("[GrabDemo] Failed to set the head position to (%lf, %lf)", pitch, yaw);
		return false;
	}

	return true;
}

bool GrabDemo::setupGrab(GrabTarget target)
{
	setFocusFace(false);
	setDepth(true);
	if (waitForHeadPos(GRAB_HEAD_PITCH) == false)
		return false;

	mTarget = target;
	return true;
}

void GrabDemo::teardownGrab(bool success)
{
	if (success)
		sendEmotion(nero_msgs::Emotion::HAPPY);
	else
		sendEmotion(nero_msgs::Emotion::SAD);

	setHead(NORMAL_HEAD_PITCH);
	setDepth(false);
	setFocusFace(true);

	mTarget = TARGET_NONE;
}

void GrabDemo::grabCb(const std_msgs::UInt8 &msg)
{
	if (mTarget != TARGET_NONE)
	{
		if (msg.data == TARGET_NONE)
			teardownGrab(false);
		else
			ROS_WARN("[GrabDemo] Already grabbing id %d", mTarget);
		return;
	}

	ROS_INFO("Setting up the grab demo.");
	if (setupGrab(GrabTarget(msg.data)) == false)
	{
		teardownGrab(false);
		ROS_ERROR("[GrabDemo] Failed to setup the grabbing.");
		return;
	}

	ROS_INFO("Creating distance with the table.");
	if (createTableDistance() == false)
	{
		teardownGrab(false);
		ROS_ERROR("[GrabDemo] Failed to create distance between the table.");
		return;
	}


	ROS_INFO("Successfully executed the grab demo.");
	teardownGrab(true);
}

void GrabDemo::headPosCb(const nero_msgs::PitchYaw &msg)
{
	mHeadPos = msg;
}

void GrabDemo::basePosCb(const nero_msgs::MotorPosition &msg)
{
	mBasePos = msg;
}

int main(int argc, char *argv[])
{
	// init ros
	ros::init(argc, argv, "PersonTracker");

	GrabDemo gd;
	ros::spin();

	return 0;
}
