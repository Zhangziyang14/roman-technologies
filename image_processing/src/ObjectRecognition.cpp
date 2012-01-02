/*
 * ArmTrajectoryPlanner.cpp
 *
 *  Created on: 2011-11-13
 *      Author: wilson
 */

#include "image_processing/ObjectRecognition.h"

ros::ServiceClient mObjectRecognizeClient;	//Service client for tabletop_object_detection
//ros::ServiceClient mCollisionmapClient;	//Service client for tabletop_collision_map_processing

void ObjectRecognition::recognizeCB(const std_msgs::Bool &msg)
{

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
	/*
	//wait for collision map processing client
	while ( !ros::service::waitForService("/tabletop_collision_map_processing/tabletop_collision_map_processing",
			ros::Duration(2.0)) && mNodeHandle.ok() )
	{
		ROS_INFO("Waiting for collision processing service to come up");
	}
	if (!mNodeHandle.ok())
		exit(0);

	mCollisionmapClient = mNodeHandle.serviceClient<tabletop_collision_map_processing::TabletopCollisionMapProcessing>("/tabletop_collision_map_processing/tabletop_collision_map_processing", true);

	//initialise subscribers
	mCommandSubscriber = mNodeHandle.subscribe("/objectRecognizeCommandTopic", 1, &ObjectRecognition::recognizeCB, this);
	 */
	//initialise publishers
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

	while(ros::ok())
	{
		if (!mObjectRecognizeClient.call(detection_call))
		{
			ROS_ERROR("Tabletop detection service failed");
			return -1;
		}
		usleep(200000);
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

	/*
	//call collision map processing
	ROS_INFO("Calling collision map processing");
	tabletop_collision_map_processing::TabletopCollisionMapProcessing processing_call;
	//pass the result of the tabletop detection
	processing_call.request.detection_result = detection_call.response.detection;
	//ask for the existing map and collision models to be reset
	processing_call.request.reset_collision_models = true;
	processing_call.request.reset_attached_models = true;
	//ask for the results to be returned in base link frame
	processing_call.request.desired_frame = "base_link";

	if (!mCollisionmapClient.call(processing_call))
	{
		ROS_ERROR("Collision map processing service failed");
		return -1;
	}
	//the collision map processor returns instances of graspable objects
	if (processing_call.response.graspable_objects.empty())
	{
		ROS_ERROR("Collision map processing returned no graspable objects");
		return -1;
	}
	 */
	ros::spin();
}
