/*
 * GrabDemo.h
 *
 *  Created on: Jun 15, 2012
 *      Author: tux
 */

#ifndef GRABDEMO_H_
#define GRABDEMO_H_

#include "ros/ros.h"

#include "std_msgs/UInt8.h"

#include "nero_msgs/GetCloud.h"
#include "nero_msgs/SetActive.h"
#include "nero_msgs/SegmentTable.h"
#include "nero_msgs/PitchYaw.h"
#include "nero_msgs/Emotion.h"
#include "nero_msgs/MotorPosition.h"

enum GrabTarget
{
	TARGET_NONE,
	TARGET_COKE,
	TARGET_FANTA,
};

#define GRAB_HEAD_PITCH 0.4
#define NORMAL_HEAD_PITCH 0.0
#define HEAD_FREE_THRESHOLD 0.01

#define BASE_FREE_THRESHOLD 0.05

#define TABLE_DISTANCE 0.7

#define MAX_WAIT 10.0

class GrabDemo
{
private:
	ros::NodeHandle mNodeHandle;

	ros::Publisher mEmotionPub;
	ros::Publisher mHeadPub;
	ros::Publisher mBasePub;

	ros::ServiceClient mCloudServerClient;
	ros::ServiceClient mForceDepthClient;
	ros::ServiceClient mSegmentTableClient;
	ros::ServiceClient mFocusFaceClient;

	ros::Subscriber mCommandSub;
	ros::Subscriber mHeadPosSub;
	ros::Subscriber mBasePosSub;

	GrabTarget mTarget;
	nero_msgs::PitchYaw mHeadPos;
	nero_msgs::MotorPosition mBasePos;

	inline void sendEmotion(uint8_t emotion) { std_msgs::UInt8 msg; msg.data = emotion; mEmotionPub.publish(msg); };
	inline void setHead(double pitch, double yaw = 0.0) { nero_msgs::PitchYaw msg; msg.pitch = pitch; msg.yaw = yaw; mHeadPub.publish(msg); };
	inline void setBase(double distance) { nero_msgs::MotorPosition msg; msg.left = distance; msg.right = distance; mBasePub.publish(msg); };
	inline void setDepth(bool on) { nero_msgs::SetActive srv; srv.request.active = on; mForceDepthClient.call(srv); };
	inline void setFocusFace(bool on) { nero_msgs::SetActive srv; srv.request.active = on; mFocusFaceClient.call(srv); };
	inline bool getCloud(sensor_msgs::PointCloud2 &cloud) { nero_msgs::GetCloud srv; if (mCloudServerClient.call(srv) == false) return false; cloud = srv.response.cloud; return true; };
	inline bool segmentTable(nero_msgs::Table &table, sensor_msgs::PointCloud2 cloud)
	{
		nero_msgs::SegmentTable srv;
		srv.request.cloud = cloud;
		if (mSegmentTableClient.call(srv) == false)
			return false;

		table = srv.response.table;
		return true;
	};

	bool setupGrab(GrabTarget target);
	void teardownGrab(bool success);

	bool waitForHeadPos(double pitch, double yaw = 0.0);
	bool waitForBasePos(double distance);

	bool createTableDistance();

public:
	GrabDemo();

	void grabCb(const std_msgs::UInt8 &msg);
	void headPosCb(const nero_msgs::PitchYaw &msg);
	void basePosCb(const nero_msgs::MotorPosition &msg);
};

#endif /* GRABDEMO_H_ */
