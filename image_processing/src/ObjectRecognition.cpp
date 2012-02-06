/*
 * ArmTrajectoryPlanner.cpp
 *
 *  Created on: 2011-11-13
 *      Author: wilson
 */

#include "image_processing/ObjectRecognition.h"

double distanceFromPose(geometry_msgs::Pose pose)
{
	return sqrt(pose.position.x*pose.position.x + pose.position.y*pose.position.y + pose.position.z*pose.position.z);
}

ObjectRecognition::ObjectRecognition() : mNodeHandle("")
{
	//wait for detection client
	while ( !ros::service::waitForService("/object_detection",
			ros::Duration(2.0)) && mNodeHandle.ok() )
	{
		ROS_INFO("Waiting for object detection service to come up");
	}
	if (!mNodeHandle.ok())
		exit(0);

	mObjectRecognizeClient = mNodeHandle.serviceClient<tabletop_object_detector::TabletopDetection>("/object_detection", true);
	
	//initialise services
	mFindObjectServer = mNodeHandle.advertiseService("/cmd_object_recognition", &ObjectRecognition::recognizeCB, this);

	mNodeHandle.param<double>("min_marker_quality", mMinQuality, 0.003);
}

bool ObjectRecognition::recognizeCB(nero_msgs::FindObject::Request &req, nero_msgs::FindObject::Response &res)
{
	res.result = res.SUCCESS;
	if (req.objectId == -1)
		return false;

	//call the tabletop detection
	ROS_INFO("Calling tabletop detector");
	tabletop_object_detector::TabletopDetection detection_call;
	//we want recognized database objects returned
	//set this to false if you are using the pipeline without the database
	detection_call.request.return_clusters = true;
	//we want the individual object point clouds returned as well
	detection_call.request.return_models = true;
	detection_call.request.num_models = 1;

	if (!mObjectRecognizeClient.call(detection_call))
	{
		res.result = res.FAILED;
		//ROS_ERROR("Tabletop detection service failed");
		//return -1;
	}
	if (detection_call.response.detection.result !=
			detection_call.response.detection.SUCCESS)
	{
		res.result = res.FAILED;
		//ROS_ERROR("Tabletop detection returned error code %d", detection_call.response.detection.result);
		//return -1;
	}
	if (detection_call.response.detection.clusters.empty() &&
			detection_call.response.detection.models.empty() )
	{
		res.result = res.FAILED;
		//ROS_ERROR("The tabletop detector dete0cted the table, but found no objects");
		//return -1;
	}

	// check results for match
	double closestDistance = 9999.9;
	for (size_t i = 0; i < detection_call.response.detection.models.size(); i++)
	{
		for (size_t j = 0; j < detection_call.response.detection.models[i].model_list.size(); j++)
		{
			household_objects_database_msgs::DatabaseModelPose pose = detection_call.response.detection.models[i].model_list[j];

			if (pose.model_id == req.objectId && pose.confidence < mMinQuality)
			{
				geometry_msgs::PoseStamped msgToArm;
				mTransformListener.transformPose("arm_frame", pose.pose, msgToArm);
				double dist = distanceFromPose(msgToArm.pose);
				if (dist < closestDistance)
				{
					res.pose = msgToArm;
					closestDistance = dist;
				}
			}
		}
	}

	if (closestDistance < 9999.0)
	{
		geometry_msgs::PoseStamped pose, poseInArmSpace;
		pose = detection_call.response.detection.table.pose;
		mTransformListener.transformPose("arm_frame", pose, poseInArmSpace);

		res.min_y = detection_call.response.detection.table.y_min;
		res.table_z = poseInArmSpace.pose.position.z;
		return true;
	}

	return false;
}

int main(int argc, char **argv)
{
	// init ros and ObjectRecognition
	ros::init(argc, argv, "ObjectRecognition");
	ObjectRecognition objectRecognition;

	int sleep_rate;
	objectRecognition.getNodeHandle()->param<int>("node_sleep_rate", sleep_rate, 50);
	ros::Rate sleep(sleep_rate);

	while(ros::ok())
	{
		sleep.sleep();
		ros::spinOnce();
	}
}
