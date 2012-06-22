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
#include "std_msgs/Float32.h"

#include "nero_msgs/GetCloud.h"
#include "nero_msgs/SetActive.h"
#include "nero_msgs/SegmentTable.h"
#include "nero_msgs/PitchYaw.h"
#include "nero_msgs/Emotion.h"
#include "nero_msgs/MotorPosition.h"
#include "nero_msgs/QueryCloud.h"

#include "tf/transform_listener.h"

enum GrabTarget
{
	TARGET_NONE,
	TARGET_COKE,
	TARGET_FANTA,
};

#define GRAB_HEAD_PITCH 0.4
#define NORMAL_HEAD_PITCH 0.0

#define HEAD_FREE_THRESHOLD 0.1
#define BASE_FREE_THRESHOLD 0.1
#define ANGLE_FREE_THRESHOLD 0.01

#define TABLE_DISTANCE 0.7

#define MAX_WAIT 10.0

#define BASE_RADIUS 			0.25	//[m]

class GrabDemo
{
private:
	ros::NodeHandle mNodeHandle;
	tf::TransformListener mTransformListener;

	ros::Publisher mEmotionPub;
	ros::Publisher mHeadPub;
	ros::Publisher mBasePub;
	ros::Publisher mCloudPub;

	ros::ServiceClient mCloudServerClient;
	ros::ServiceClient mForceDepthClient;
	ros::ServiceClient mSegmentTableClient;
	ros::ServiceClient mFocusFaceClient;
	ros::ServiceClient mQueryCloudClient;

	ros::Subscriber mCommandSub;
	ros::Subscriber mHeadPosSub;
	ros::Subscriber mBasePosSub;
	ros::Subscriber mObjPosSub;

	GrabTarget mTarget;
	nero_msgs::PitchYaw mHeadPos;
	nero_msgs::MotorPosition mBasePos;
	geometry_msgs::PointStamped mObjPos;

	inline void sendEmotion(uint8_t emotion) { std_msgs::UInt8 msg; msg.data = emotion; mEmotionPub.publish(msg); };
	inline void setHead(double pitch, double yaw = 0.0) { nero_msgs::PitchYaw msg; msg.pitch = pitch; msg.yaw = yaw; mHeadPub.publish(msg); };
	inline void setBase(double left, double right) { nero_msgs::MotorPosition msg; msg.left = left; msg.right = right; mBasePub.publish(msg); };
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
	inline bool getRealWorldPoint(geometry_msgs::Point pin, geometry_msgs::PointStamped &pout)
	{
		nero_msgs::QueryCloud srv;
		srv.request.points.push_back(pin);
		if (mQueryCloudClient.call(srv) == false)
			return false;

		mTransformListener.waitForTransform(srv.response.points[0].header.frame_id, "/base_link", srv.response.points[0].header.stamp, ros::Duration(2.0));
		mTransformListener.transformPoint("/arm_frame", srv.response.points[0], pout);
		return true;
	};

	bool setupGrab(GrabTarget target);
	void teardownGrab(bool success);

	bool waitForHeadPos(double pitch, double yaw = 0.0);
	bool waitForBasePos(double left, double right, double threshold);

	bool createTableDistance();
	bool rotateToObject();

public:
	GrabDemo();

	void grabCb(const std_msgs::UInt8 &msg);
	void headPosCb(const nero_msgs::PitchYaw &msg);
	void basePosCb(const nero_msgs::MotorPosition &msg);
	void objPosCb(const geometry_msgs::PointStamped &msg);
};

#endif /* GRABDEMO_H_ */
