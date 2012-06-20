/*
 * TabletopSegmentation.h
 *
 *  Created on: Jun 19, 2012
 *      Author: tux
 */

#ifndef TABLETOPSEGMENTATION_H_
#define TABLETOPSEGMENTATION_H_

#include "ros/ros.h"

#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/PointCloud2.h"
#include "nero_msgs/SegmentTable.h"

#include "tf/transform_listener.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/segmentation/extract_clusters.h>

typedef pcl::PointXYZ    Point;
typedef pcl::search::KdTree<Point>::Ptr KdTreePtr;

class TabletopSegmentation
{
private:
	ros::NodeHandle mNodeHandle;
	ros::ServiceServer mSegmentServer;

	double mMinZ;
	double mMaxZ;
	double mMinTableZ;
	double mMaxTableZ;
	double mPlaneVoxSize;
	double mClustVoxSize;
	double mClustDist;
	int mMinClustSize;
	int mInlierThresh;
	double mUpDirection;

	template <typename PointT>
	bool getPlanePoints(const pcl::PointCloud<PointT> &table, const tf::Transform& table_plane_trans, sensor_msgs::PointCloud &table_points);

	template <class PointCloudType>
	nero_msgs::Table getTable(std_msgs::Header cloud_header, const tf::Transform &table_plane_trans, const PointCloudType &table_points);

public:
	TabletopSegmentation();

	bool segmentCb(nero_msgs::SegmentTable::Request &req, nero_msgs::SegmentTable::Response &res);
};

#endif /* TABLETOPSEGMENTATION_H_ */
