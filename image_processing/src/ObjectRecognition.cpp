/*
 * ArmTrajectoryPlanner.cpp
 *
 *  Created on: 2011-11-13
 *      Author: wilson
 */

#include "image_processing/ObjectRecognition.h"

ros::ServiceClient mObjectRecognizeClient;		//Service client for tabletop_object_detection

void ObjectRecognition::recognizeCB(const std_msgs::UInt8 &msg)
{
	mObjectToFind = msg.data;
}

void ObjectRecognition::init()
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
}

int main(int argc, char **argv)
{
	// init ros and ObjectRecognition
	ros::init(argc, argv, "ObjectRecognition");
	ObjectRecognition objectRecognition;

	char *path = NULL;
	if (argc == 2)
		path = argv[1];

	objectRecognition.init();

	//call the tabletop detection
	ROS_INFO("Calling tabletop detector");
	tabletop_object_detector::TabletopDetection detection_call;
	//we want recognized database objects returned
	//set this to false if you are using the pipeline without the database
	detection_call.request.return_clusters = true;
	//we want the individual object point clouds returned as well
	detection_call.request.return_models = false;
	detection_call.request.num_models = 1;

	while(ros::ok() && objectRecognition.getObjectID() != -1)
	{
		if (!mObjectRecognizeClient.call(detection_call))
		{
			ROS_ERROR("Tabletop detection service failed");
			return -1;
		}

		ros::spinOnce();
		//usleep(200000);
	}
	if (detection_call.response.detection.result !=
			detection_call.response.detection.SUCCESS)
	{
		ROS_ERROR("Tabletop detection returned error code %d",
				detection_call.response.detection.result);
		return -1;
	}
	if (detection_call.response.detection.clusters.empty() &&
			detection_call.response.detection.models.empty() )
	{
		ROS_ERROR("The tabletop detector detected the table, "
				"but found no objects");
		return -1;
	}
	household_objects_database_msgs::DatabaseModelPoseList result = (household_objects_database_msgs::DatabaseModelPoseList) detection_call.response.detection.models.front(); //...?
	geometry_msgs::PoseStamped result_pose;

	//TODO loop through detection_call.response.detection.models and look for the model with mObjectToFind as ID. Publish the Stamped Pose of this model.
	for(size_t i = 0; i < result.model_list.size(); i++)
	{
		if(household_objects_database_msgs::DatabaseModelPose (result.model_list.at(i)).model_id == objectRecognition.getObjectID())
			objectRecognition.publishObjectPose(result.model_list.at(i).pose);
	}
	/*
	pcl::PointCloud<pcl::PointXYZ> cloud;
	sensor_msgs::PointCloud2 converted_point_cloud;
	for(size_t i=0; i < detection_call.response.detection.clusters.size(); i++)
	{
		sensor_msgs::convertPointCloudToPointCloud2(detection_call.response.detection.clusters[i], converted_point_cloud);
		pcl::fromROSMsg(converted_point_cloud, cloud);
		std::stringstream ss("cluster_");
		ss << i << ".pcd";
		pcl::io::savePCDFileASCII (ss.str().c_str(), cloud);
	}*/
}
