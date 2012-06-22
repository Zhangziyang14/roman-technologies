/*
 * TabletopSegmentation.cpp
 *
 *  Created on: Jun 19, 2012
 *      Author: tux
 */

#include "image_processing/TabletopSegmentation.h"

TabletopSegmentation::TabletopSegmentation() :
	mCurrentMarkerId(0),
	mMarkersPublished(0)
{
	mSegmentServer = mNodeHandle.advertiseService("/TabletopSegmentation/segment_table", &TabletopSegmentation::segmentCb, this);
	mMarkerPub = mNodeHandle.advertise<visualization_msgs::Marker>("/TabletopSegmentation/markers_out", 10);

	mNodeHandle.param<double>("/TabletopSegmentation/min_z", mMinZ, 0.2);
	mNodeHandle.param<double>("/TabletopSegmentation/max_z", mMaxZ, 1.25);
	mNodeHandle.param<double>("/TabletopSegmentation/min_table_z", mMinTableZ, 0.01);
	mNodeHandle.param<double>("/TabletopSegmentation/max_table_z", mMaxTableZ, 0.50);
	mNodeHandle.param<double>("/TabletopSegmentation/plane_detection_voxel_size", mPlaneVoxSize, 0.001);
	mNodeHandle.param<double>("/TabletopSegmentation/clustering_voxel_size", mClustVoxSize, 0.003);
	mNodeHandle.param<double>("/TabletopSegmentation/clustering_distance", mClustDist, 0.03);
	mNodeHandle.param<int>("/TabletopSegmentation/min_cluster_size", mMinClustSize, 300);
	mNodeHandle.param<int>("/TabletopSegmentation/inlier_threshold", mInlierThresh, 300);
	mNodeHandle.param<double>("/TabletopSegmentation/up_direction", mUpDirection, 1.0);
	mNodeHandle.param<std::string>("/TabletopSegmentation/processing_frame", mProcessingFrame, "/base_link");

	ROS_INFO("TabletopSegmentation initialized.");
}

void TabletopSegmentation::clearOldMarkers(std::string frame_id)
{
	for (int id = mCurrentMarkerId; id < mMarkersPublished; id++)
	{
		visualization_msgs::Marker delete_marker;
		delete_marker.header.stamp = ros::Time::now();
		delete_marker.header.frame_id = frame_id;
		delete_marker.id = id;
		delete_marker.action = visualization_msgs::Marker::DELETE;
		delete_marker.ns = "TabletopSegmentation";
		mMarkerPub.publish(delete_marker);
	}

	mMarkersPublished = mCurrentMarkerId;
	mCurrentMarkerId = 0;
}

visualization_msgs::Marker TabletopSegmentation::getTableMarker(float xmin, float xmax, float ymin, float ymax)
{
	visualization_msgs::Marker marker;
	marker.action = visualization_msgs::Marker::ADD;
	marker.type = visualization_msgs::Marker::LINE_STRIP;
	marker.lifetime = ros::Duration();

	//create the marker in the table reference frame
	//the caller is responsible for setting the pose of the marker to match

	marker.scale.x = 0.001;
	marker.scale.y = 0.001;
	marker.scale.z = 0.001;

	marker.points.resize(5);
	marker.points[0].x = xmin;
	marker.points[0].y = ymin;
	marker.points[0].z = 0;

	marker.points[1].x = xmin;
	marker.points[1].y = ymax;
	marker.points[1].z = 0;

	marker.points[2].x = xmax;
	marker.points[2].y = ymax;
	marker.points[2].z = 0;

	marker.points[3].x = xmax;
	marker.points[3].y = ymin;
	marker.points[3].z = 0;

	marker.points[4].x = xmin;
	marker.points[4].y = ymin;
	marker.points[4].z = 0;

	marker.points.resize(6);
	marker.points[5].x = xmin;
	marker.points[5].y = ymin;
	marker.points[5].z = 0.02;

	marker.color.r = 1.0;
	marker.color.g = 1.0;
	marker.color.b = 0.0;
	marker.color.a = 1.0;

	return marker;
}

template <typename PointT>
bool TabletopSegmentation::getPlanePoints(const pcl::PointCloud<PointT> &table, const tf::Transform& table_plane_trans, sensor_msgs::PointCloud &table_points)
{
	// Prepare the output
	table_points.header = table.header;
	table_points.points.resize (table.points.size ());
	for (size_t i = 0; i < table.points.size (); ++i)
	{
		table_points.points[i].x = table.points[i].x;
		table_points.points[i].y = table.points[i].y;
		table_points.points[i].z = table.points[i].z;
	}

	// Transform the data
	tf::StampedTransform table_pose_frame(table_plane_trans, table.header.stamp, table.header.frame_id, "table_frame");
	mTransformListener.setTransform(table_pose_frame);
	std::string error_msg;
	if (!mTransformListener.canTransform("table_frame", table_points.header.frame_id, table_points.header.stamp, &error_msg))
	{
		ROS_ERROR("Can not transform point cloud from frame %s to table frame; error %s",
		table_points.header.frame_id.c_str(), error_msg.c_str());
		return false;
	}

	try
	{
		mTransformListener.transformPointCloud("table_frame", table_points, table_points);
	}
	catch (tf::TransformException ex)
	{
		ROS_ERROR("Failed to transform point cloud from frame %s into table_frame; error %s",
		table_points.header.frame_id.c_str(), ex.what());
		return false;
	}

	table_points.header.stamp = table.header.stamp;
	table_points.header.frame_id = "table_frame";
	return true;
}

template <class PointCloudType>
nero_msgs::Table TabletopSegmentation::getTable(std_msgs::Header cloud_header, const tf::Transform &table_plane_trans, const PointCloudType &table_points)
{
	nero_msgs::Table table;

	//get the extents of the table
	if (!table_points.points.empty())
	{
		table.x_min = table_points.points[0].x;
		table.x_max = table_points.points[0].x;
		table.y_min = table_points.points[0].y;
		table.y_max = table_points.points[0].y;
	}

	for (size_t i=1; i<table_points.points.size(); ++i)
	{
		if (table_points.points[i].x<table.x_min && table_points.points[i].x>-3.0) table.x_min = table_points.points[i].x;
		if (table_points.points[i].x>table.x_max && table_points.points[i].x< 3.0) table.x_max = table_points.points[i].x;
		if (table_points.points[i].y<table.y_min && table_points.points[i].y>-3.0) table.y_min = table_points.points[i].y;
		if (table_points.points[i].y>table.y_max && table_points.points[i].y< 3.0) table.y_max = table_points.points[i].y;
	}

	geometry_msgs::Pose table_pose;
	tf::poseTFToMsg(table_plane_trans, table_pose);
	table.pose.pose = table_pose;
	table.pose.header = cloud_header;

	visualization_msgs::Marker tableMarker = getTableMarker(table.x_min, table.x_max, table.y_min, table.y_max);
	tableMarker.header = cloud_header;
	tableMarker.pose = table_pose;
	tableMarker.ns = "TabletopSegmentation";
	tableMarker.id = mCurrentMarkerId++;
	mMarkerPub.publish(tableMarker);

	return table;
}

tf::Transform getPlaneTransform(pcl::ModelCoefficients coeffs, double up_direction)
{
	ROS_ASSERT(coeffs.values.size() > 3);
	double a = coeffs.values[0], b = coeffs.values[1], c = coeffs.values[2], d = coeffs.values[3];
	//asume plane coefficients are normalized
	btVector3 position(-a*d, -b*d, -c*d);
	btVector3 z(a, b, c);
	//make sure z points "up"
	ROS_DEBUG("z.dot: %0.3f", z.dot(btVector3(0,0,1)));
	ROS_DEBUG("in getPlaneTransform, z: %0.3f, %0.3f, %0.3f", z[0], z[1], z[2]);
	if ( z.dot( btVector3(0, 0, up_direction) ) < 0)
	{
		z = -1.0 * z;
		ROS_INFO("flipped z");
	}
	ROS_DEBUG("in getPlaneTransform, z: %0.3f, %0.3f, %0.3f", z[0], z[1], z[2]);

	//try to align the x axis with the x axis of the original frame
	//or the y axis if z and x are too close too each other
	btVector3 x(1, 0, 0);
	if ( fabs(z.dot(x)) > 1.0 - 1.0e-4) x = btVector3(0, 1, 0);
	btVector3 y = z.cross(x).normalized();
	x = y.cross(z).normalized();

	btMatrix3x3 rotation;
	rotation[0] = x; 	// x
	rotation[1] = y; 	// y
	rotation[2] = z; 	// z
	rotation = rotation.transpose();
	btQuaternion orientation;
	rotation.getRotation(orientation);
	return tf::Transform(orientation, position);
}

bool TabletopSegmentation::segmentCb(nero_msgs::SegmentTable::Request &req, nero_msgs::SegmentTable::Response &res)
{
	ROS_INFO("Starting process on new cloud");
	ROS_INFO("In frame %s", req.cloud.header.frame_id.c_str());

	sensor_msgs::PointCloud2 cloud;
	if (mProcessingFrame != "")
	{
		//convert cloud to base link frame
		sensor_msgs::PointCloud old_cloud;
		sensor_msgs::convertPointCloud2ToPointCloud(req.cloud, old_cloud);
		try
		{
			old_cloud.header.stamp = ros::Time::now(); // there might have been some delay, make this an updated cloud
			mTransformListener.waitForTransform(old_cloud.header.frame_id, mProcessingFrame, old_cloud.header.stamp, ros::Duration(2.0));
			mTransformListener.transformPointCloud(mProcessingFrame, old_cloud, old_cloud);
		}
		catch (tf::TransformException &ex)
		{
			ROS_ERROR("Failed to transform cloud from frame %s into frame %s; %s", old_cloud.header.frame_id.c_str(), mProcessingFrame.c_str(), ex.what());
			return false;
		}

		sensor_msgs::convertPointCloudToPointCloud2(old_cloud, cloud);
		ROS_INFO("Input cloud converted to %s frame", mProcessingFrame.c_str());
		clearOldMarkers(cloud.header.frame_id);
	}
	else
	{
		cloud = req.cloud;
		clearOldMarkers(cloud.header.frame_id);
	}

	// PCL objects
	KdTreePtr normals_tree_, clusters_tree_;
	pcl::VoxelGrid<Point> grid_, grid_objects_;
	pcl::PassThrough<Point> pass_;
	pcl::NormalEstimation<Point, pcl::Normal> n3d_;
	pcl::SACSegmentationFromNormals<Point, pcl::Normal> seg_;
	pcl::ProjectInliers<Point> proj_;
	pcl::ConvexHull<Point> hull_;
	pcl::ExtractPolygonalPrismData<Point> prism_;
	pcl::EuclideanClusterExtraction<Point> pcl_cluster_;

	// Filtering parameters
	grid_.setLeafSize(mPlaneVoxSize, mPlaneVoxSize, mPlaneVoxSize);
	grid_objects_.setLeafSize(mClustVoxSize, mClustVoxSize, mClustVoxSize);
	grid_.setFilterFieldName ("z");
	pass_.setFilterFieldName ("z");

	pass_.setFilterLimits(mMinZ, mMaxZ);
	grid_.setFilterLimits(mMinZ, mMaxZ);
	grid_.setDownsampleAllData(false);
	grid_objects_.setDownsampleAllData(false);

	normals_tree_ = boost::make_shared<pcl::search::KdTree<Point> > ();
	clusters_tree_ = boost::make_shared<pcl::search::KdTree<Point> > ();

	// Normal estimation parameters
	n3d_.setKSearch (10);
	n3d_.setSearchMethod (normals_tree_);
	// Table model fitting parameters
	seg_.setDistanceThreshold (0.05);
	seg_.setMaxIterations (10000);
	seg_.setNormalDistanceWeight (0.1);
	seg_.setOptimizeCoefficients (true);
	seg_.setModelType (pcl::SACMODEL_NORMAL_PLANE);
	seg_.setMethodType (pcl::SAC_RANSAC);
	seg_.setProbability (0.99);

	proj_.setModelType (pcl::SACMODEL_PLANE);

	// Clustering parameters
	pcl_cluster_.setClusterTolerance (mClustDist);
	pcl_cluster_.setMinClusterSize (mMinClustSize);
	pcl_cluster_.setSearchMethod (clusters_tree_);

	// Step 1 : Filter, remove NaNs and downsample
	pcl::PointCloud<Point> cloud_t;
	pcl::fromROSMsg (cloud, cloud_t);
	pcl::PointCloud<Point>::ConstPtr cloud_ptr = boost::make_shared<const pcl::PointCloud<Point> > (cloud_t);

	pcl::PointCloud<Point> cloud_filtered;
	pass_.setInputCloud (cloud_ptr);
	pass_.filter (cloud_filtered);
	pcl::PointCloud<Point>::ConstPtr cloud_filtered_ptr =
	boost::make_shared<const pcl::PointCloud<Point> > (cloud_filtered);
	ROS_INFO("Step 1 done");

	if (cloud_filtered.points.size() < (unsigned int)mMinClustSize)
	{
		ROS_WARN("Filtered cloud only has %d points", (int)cloud_filtered.points.size());
		return false;
	}

	pcl::PointCloud<Point> cloud_downsampled;
	grid_.setInputCloud (cloud_filtered_ptr);
	grid_.filter (cloud_downsampled);
	pcl::PointCloud<Point>::ConstPtr cloud_downsampled_ptr = boost::make_shared<const pcl::PointCloud<Point> > (cloud_downsampled);

	if (cloud_downsampled.points.size() < (unsigned int)mMinClustSize)
	{
		ROS_WARN("Downsampled cloud only has %d points", (int)cloud_downsampled.points.size());
		return false;
	}

	// Step 2 : Estimate normals
	pcl::PointCloud<pcl::Normal> cloud_normals;
	n3d_.setInputCloud (cloud_downsampled_ptr);
	n3d_.compute (cloud_normals);
	pcl::PointCloud<pcl::Normal>::ConstPtr cloud_normals_ptr =
	boost::make_shared<const pcl::PointCloud<pcl::Normal> > (cloud_normals);
	ROS_INFO("Step 2 done");

	// Step 3 : Perform planar segmentation
	pcl::PointIndices table_inliers; pcl::ModelCoefficients table_coefficients;
	seg_.setInputCloud (cloud_downsampled_ptr);
	seg_.setInputNormals (cloud_normals_ptr);
	seg_.segment (table_inliers, table_coefficients);
	pcl::PointIndices::ConstPtr table_inliers_ptr = boost::make_shared<const pcl::PointIndices> (table_inliers);
	pcl::ModelCoefficients::ConstPtr table_coefficients_ptr =
	boost::make_shared<const pcl::ModelCoefficients> (table_coefficients);

	if (table_coefficients.values.size () <=3)
	{
		ROS_WARN("Failed to detect table in scan");
		return false;
	}

	if (table_inliers.indices.size() < (unsigned int)mInlierThresh)
	{
		ROS_WARN("Plane detection has %d inliers, below min threshold of %d", (int)table_inliers.indices.size(), mInlierThresh);
		return false;
	}

	ROS_INFO ("[TableObjectDetector::input_callback] Model found with %d inliers: [%f %f %f %f].",
		(int)table_inliers.indices.size (),
		table_coefficients.values[0], table_coefficients.values[1],
		table_coefficients.values[2], table_coefficients.values[3]);
	ROS_INFO("Step 3 done");

	// Step 4 : Project the table inliers on the table
	pcl::PointCloud<Point> table_projected;
	proj_.setInputCloud (cloud_downsampled_ptr);
	proj_.setIndices (table_inliers_ptr);
	proj_.setModelCoefficients (table_coefficients_ptr);
	proj_.filter (table_projected);
	pcl::PointCloud<Point>::ConstPtr table_projected_ptr =
	boost::make_shared<const pcl::PointCloud<Point> > (table_projected);
	ROS_INFO("Step 4 done");

	sensor_msgs::PointCloud table_points;
	tf::Transform table_plane_trans = getPlaneTransform(table_coefficients, mUpDirection);
	//takes the points projected on the table and transforms them into the PointCloud message
	//while also transforming them into the table's coordinate system
	if (!getPlanePoints<Point> (table_projected, table_plane_trans, table_points))
	{
		ROS_WARN("Failed to retrieve plane points.");
		return false;
	}
	ROS_INFO("Table computed");

	res.table = getTable<sensor_msgs::PointCloud>(cloud.header, table_plane_trans, table_points);
	return true;
}

int main(int argc, char *argv[])
{
	// init ros
	ros::init(argc, argv, "TabletopSegmentation");
	srand(0);

	TabletopSegmentation ts;
	ros::spin();

	return 0;
}
