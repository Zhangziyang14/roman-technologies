/*
 * GrabDemo.cpp
 *
 *  Created on: Jun 15, 2012
 *      Author: tux
 */


#include "logical_unit/GrabDemo.h"

GrabDemo::GrabDemo() :
	mTransformListener(mNodeHandle),
	mTarget(TARGET_NONE)
{
	mHeadPub			= mNodeHandle.advertise<nero_msgs::PitchYaw>("/cmd_head_position", 1);
	mBasePub			= mNodeHandle.advertise<nero_msgs::MotorPosition>("/positionTopic", 1);
	mEmotionPub			= mNodeHandle.advertise<std_msgs::UInt8>("/cmd_emotion", 1);
	mCloudPub			= mNodeHandle.advertise<sensor_msgs::PointCloud2>("/camera/depth/points", 1);

	mCloudServerClient	= mNodeHandle.serviceClient<nero_msgs::GetCloud>("/KinectServer/CloudServer", true);
	mForceDepthClient	= mNodeHandle.serviceClient<nero_msgs::SetActive>("/KinectServer/ForceDepth", true);
	mSegmentTableClient	= mNodeHandle.serviceClient<nero_msgs::SegmentTable>("/TabletopSegmentation/segment_table", true);
	mFocusFaceClient	= mNodeHandle.serviceClient<nero_msgs::SetActive>("/set_focus_face", true);
	mQueryCloudClient	= mNodeHandle.serviceClient<nero_msgs::QueryCloud>("/KinectServer/QueryCloud", true);

	mCommandSub			= mNodeHandle.subscribe("/GrabDemo/grab_id", 1, &GrabDemo::grabCb, this);
	mHeadPosSub			= mNodeHandle.subscribe("/headPositionFeedbackTopic", 1, &GrabDemo::headPosCb, this);
	mBasePosSub			= mNodeHandle.subscribe("/MobileBase/position", 1, &GrabDemo::basePosCb, this);
	mObjPosSub			= mNodeHandle.subscribe("/ObjectTracking/object_position", 1, &GrabDemo::objPosCb, this);

	mHeadPos.pitch = 0.0;
	mHeadPos.yaw = 0.0;
	mBasePos.left = 0.0;
	mBasePos.right = 0.0;
	mObjPos.point.x = -1.0;
	mObjPos.point.y = -1.0;
	mObjPos.point.z = -1.0;

	ROS_INFO("[GrabDemo] Initialized.");
}

bool GrabDemo::rotateToObject()
{
	ros::Rate rate(50);

	while (ros::ok())
	{
		if (mObjPos.point.x >= 0.0 && mObjPos.point.y >= 0.0 &&
			isnan(mObjPos.point.x) == false && isnan(mObjPos.point.y) == false)
		{
			geometry_msgs::PointStamped pos;
			if (getRealWorldPoint(mObjPos.point, pos) == false)
			{
				ROS_ERROR("[GrabDemo] Failed to query real world point.");
				return false;
			}

			double angle = -atan(pos.point.x / pos.point.y);
			if (fabs(angle) < ANGLE_FREE_THRESHOLD)
				break;

			ROS_INFO("x: %lf, y: %lf, angle: %lf", pos.point.x, pos.point.y, angle);

			// turning with the clock = negative
			double left = -1 * angle * BASE_RADIUS;
			double right = angle * BASE_RADIUS;

			if (waitForBasePos(left, right, ANGLE_FREE_THRESHOLD) == false)
			{
				ROS_ERROR("[GrabDemo] Failed to rotate to object position.");
				return false;
			}
		}

		ros::spinOnce();
		rate.sleep();
	}
	return true;
}

bool GrabDemo::createTableDistance()
{
	while (ros::ok())
	{
		sensor_msgs::PointCloud2 pc;
		if (getCloud(pc) == false)
		{
			ROS_ERROR("[GrabDemo] Failed to get the PointCloud.");
			return false;
		}

		ROS_INFO("[GrabDemo] Received PointCloud, segmenting table...");
		mCloudPub.publish(pc);

		nero_msgs::Table table;
		if (segmentTable(table, pc) == false)
		{
			ROS_ERROR("[GrabDemo] Failed to segment the table.");
			return false;
		}

		double dist = table.y_min - TABLE_DISTANCE;
		if (fabs(dist) < BASE_FREE_THRESHOLD)
			break;

		if (waitForBasePos(dist, dist, BASE_FREE_THRESHOLD) == false)
			return false;
	}

	return true;
}

bool GrabDemo::waitForBasePos(double left, double right, double threshold)
{
	ROS_INFO("waitForBasePos call");
	ros::Rate rate(10);
	double start = ros::Time::now().toSec();

	nero_msgs::MotorPosition startPos = mBasePos;
	setBase(left, right);

	while (ros::ok())// && ros::Time::now().toSec() - start < MAX_WAIT)
	{
		ROS_INFO("mBasePos.left: %lf, startPos.left: %lf, left: %lf", mBasePos.left, startPos.left, left);
		if (fabs(mBasePos.left - startPos.left - left) < threshold &&
			fabs(mBasePos.right - startPos.right - right) < threshold)
			break;

		ros::spinOnce();
		rate.sleep();
	}

	if (ros::Time::now().toSec() - start >= MAX_WAIT)
	{
		ROS_ERROR("[GrabDemo] Failed to set the base with distance (%lf, %lf)", left, right);
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
	//else
	//	sendEmotion(nero_msgs::Emotion::SAD);

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

	ROS_INFO("[GrabDemo] Setting up the grab demo.");
	if (setupGrab(GrabTarget(msg.data)) == false)
	{
		teardownGrab(false);
		ROS_ERROR("[GrabDemo] Failed to setup the grabbing.");
		return;
	}

	/*ROS_INFO("Creating distance with the table.");
	if (createTableDistance() == false)
	{
		teardownGrab(false);
		ROS_ERROR("[GrabDemo] Failed to create distance between the table.");
		return;
	}*/

	ROS_INFO("[GrabDemo] Rotating to object location.");
	if (rotateToObject() == false)
	{
		teardownGrab(false);
		ROS_ERROR("[GrabDemo] Failed to rotate to object location.");
		return;
	}


	ROS_INFO("[GrabDemo] Successfully executed the grab demo.");
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

void GrabDemo::objPosCb(const geometry_msgs::PointStamped &msg)
{
	mObjPos = msg;
}

int main(int argc, char *argv[])
{
	// init ros
	ros::init(argc, argv, "PersonTracker");

	GrabDemo gd;
	ros::spin();

	return 0;
}
