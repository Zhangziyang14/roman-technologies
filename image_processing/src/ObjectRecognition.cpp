/*
 * ArmTrajectoryPlanner.cpp
 *
 *  Created on: 2011-11-13
 *      Author: wilson
 */

#include "image_processing/ObjectRecognition.h"

ros::ServiceClient mObjectRecognizeClient;		//Service client for tabletop_object_detection

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
	
	//initialise subscribers
	mCommandSubscriber = mNodeHandle.subscribe("/cmd_object_recognition", 1, &ObjectRecognition::recognizeCB, this);
	
	//initialise publishers
	mObjectPosePublisher = mNodeHandle.advertise<geometry_msgs::PoseStamped>("/objectPoseFeedbackTopic", 1);

	mObjectToFind = -1;
	mNodeHandle.param<double>("min_marker_quality", mMinQuality, 0.003);
}

void ObjectRecognition::recognizeCB(const std_msgs::Int32 &msg)
{
	mObjectToFind = msg.data;
}

void ObjectRecognition::spin()
{
	//call the tabletop detection
	ROS_INFO("Calling tabletop detector");
	tabletop_object_detector::TabletopDetection detection_call;
	//we want recognized database objects returned
	//set this to false if you are using the pipeline without the database
	detection_call.request.return_clusters = true;
	//we want the individual object point clouds returned as well
	detection_call.request.return_models = true;
	detection_call.request.num_models = 1;

	while (ros::ok())
	{
		// nothing to do when objectID = -1
		while (getObjectID() == -1)
		{
			usleep(200000);
			ros::spinOnce();
			if (ros::ok() == false)
				return;
		}

		if (!mObjectRecognizeClient.call(detection_call))
		{
			ROS_ERROR("Tabletop detection service failed");
			//return -1;
		}
		if (detection_call.response.detection.result !=
				detection_call.response.detection.SUCCESS)
		{
			ROS_ERROR("Tabletop detection returned error code %d",
					detection_call.response.detection.result);
			//return -1;
		}
		if (detection_call.response.detection.clusters.empty() &&
				detection_call.response.detection.models.empty() )
		{
			ROS_ERROR("The tabletop detector detected the table, "
					"but found no objects");
			//return -1;
		}

		// check results for match
		for (size_t i = 0; i < detection_call.response.detection.models.size(); i++)
		{
			for (size_t j = 0; j < detection_call.response.detection.models[i].model_list.size(); j++)
			{
				household_objects_database_msgs::DatabaseModelPose pose = detection_call.response.detection.models[i].model_list[j];
				if (pose.model_id == getObjectID() && pose.confidence < mMinQuality)
					publishObjectPose(pose.pose);
			}
		}

		ros::spinOnce();
	}
}

int main(int argc, char **argv)
{
	// init ros and ObjectRecognition
	ros::init(argc, argv, "ObjectRecognition");
	ObjectRecognition objectRecognition;
	objectRecognition.spin();
}
