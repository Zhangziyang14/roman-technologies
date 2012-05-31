/*
 * PathPlanner.cpp
 *
 *  Created on: Nov 14, 2011
 *      Author: hans
 */

#include "logical_unit/PathPlanner.h"

/**
 * PathPlanner constructor.
 */
PathPlanner::PathPlanner(const char *navmesh_file) :
	mNodeHandle("~"),
	mNavMesh(NULL),
	mNavMeshQuery(NULL),
	mTileCache(NULL)
{
	mTileCachePath = navmesh_file;
	loadNavmesh();

	mNavMeshQuery = dtAllocNavMeshQuery();
	mNavMeshQuery->init(mNavMesh, 2048);

	// this is used for enabling swimming, climbing and what not ... our robot will not do this :)
	mFilter.setIncludeFlags(0xffff ^ 0x10);
	mFilter.setExcludeFlags(0);

	std::string goalTopic, pathTopic, scanTopic;
	mNodeHandle.param<std::string>("goal_topic", goalTopic, "/move_base_simple/goal");
	mNodeHandle.param<std::string>("path_topic", pathTopic, "/global_path");
	mNodeHandle.param<std::string>("scan_topic", scanTopic, "/scan");
	mGoalSub = mNodeHandle.subscribe(goalTopic, 1, &PathPlanner::goalCB, this);
	mPathPub = mNodeHandle.advertise<nav_msgs::Path>(pathTopic, 1);
	mScanSub = mNodeHandle.subscribe(scanTopic, 1, &PathPlanner::scanCB, this);
}

/**
 * PathPlanner destructor.
 */
PathPlanner::~PathPlanner()
{
	dtFreeNavMesh(mNavMesh);
	dtFreeNavMeshQuery(mNavMeshQuery);
	dtFreeTileCache(mTileCache);
}

/**
 * Loads navigation mesh into memory.
 */
void PathPlanner::loadNavmesh()
{
	FILE* fp = fopen(mTileCachePath.c_str(), "rb");
	if (fp == NULL)
	{
		ROS_INFO("Navmesh binary could not be found. Path: %s", mTileCachePath.c_str());
		return;
	}

	// Read header.
	TileCacheSetHeader header;
	int bytes = fread(&header, sizeof(TileCacheSetHeader), 1, fp);
	if (header.magic != TILECACHESET_MAGIC)
	{
		ROS_INFO("Magic number does not match");
		fclose(fp);
		return;
	}
	if (header.version != TILECACHESET_VERSION)
	{
		ROS_INFO("Version number does not match");
		fclose(fp);
		return;
	}

	if (mNavMesh)
		dtFreeNavMesh(mNavMesh);
	mNavMesh = dtAllocNavMesh();
	if (mNavMesh == NULL)
	{
		ROS_INFO("NavMesh could not be allocated.");
		fclose(fp);
		return;
	}
	dtStatus status = mNavMesh->init(&header.meshParams);
	if (dtStatusFailed(status))
	{
		ROS_INFO("Mesh parameters init has failed.");
		fclose(fp);
		return;
	}

	mTalloc = new LinearAllocator(32000);
	mTComp = new FastLZCompressor;
	mTmpProc = new MeshProcess;

	mTileCache = dtAllocTileCache();
	status = mTileCache->init(&header.cacheParams, mTalloc, mTComp, mTmpProc);
	if (dtStatusFailed(status))
	{
		ROS_INFO("TileCache parameters init has failed.");
		fclose(fp);
		return;
	}

	ROS_INFO("Max objects: %d", header.cacheParams.maxObstacles);

	// Read tiles.
	for (int i = 0; i < header.numTiles; ++i)
	{
		TileCacheTileHeader tileHeader;
		bytes = fread(&tileHeader, sizeof(tileHeader), 1, fp);
		if (!tileHeader.tileRef || !tileHeader.dataSize)
			break;

		unsigned char* data = (unsigned char*)dtAlloc(tileHeader.dataSize, DT_ALLOC_PERM);
		if (!data) break;
		memset(data, 0, tileHeader.dataSize);
		bytes = fread(data, tileHeader.dataSize, 1, fp);

		dtCompressedTileRef tile = 0;
		mTileCache->addTile(data, tileHeader.dataSize, DT_COMPRESSEDTILE_FREE_DATA, &tile);

		if (tile)
			mTileCache->buildNavMeshTile(tile, mNavMesh);
	}

	fclose(fp);
}

void PathPlanner::addObstacle(geometry_msgs::Point p)
{
	if (mTileCache == NULL)
		return;

	float pos[3];
	pos[0] = p.x;
	pos[1] = p.z;
	pos[2] = p.y;
	if (dtStatus status = mTileCache->addObstacle(pos, ROBOT_RADIUS, 3.0, 0) != DT_SUCCESS)
		ROS_ERROR("Something went wrong adding an obstacle! %d", status);

	//ROS_INFO("Added obstacle at (%lf, %lf).", p.x, p.y);
}

void PathPlanner::removeAllObstacles()
{
	if (mTileCache == NULL)
		return;

	for (int i = 0; i < mTileCache->getObstacleCount(); i++)
	{
		const dtTileCacheObstacle* ob = mTileCache->getObstacle(i);
		if (ob->state == DT_OBSTACLE_EMPTY)
			continue;

		if (mTileCache->getObstacleRef(ob) == 0)
			ROS_WARN("Warning: Obstacle ref == 0");
		if (dtStatus status = mTileCache->removeObstacle(mTileCache->getObstacleRef(ob)) != DT_SUCCESS)
			ROS_ERROR("Something went wrong removing an obstacle! %d", status);
	}

	// update the navmesh
	float f = 0.05f;
	if (dtStatus status = mTileCache->update(f, mNavMesh) != DT_SUCCESS)
		ROS_ERROR("Something went wrong updating the tile cache! %d", status);
}

/**
 * Plans a path from start to end using the navigation mesh and Detour, putting the result in path.
 */
void PathPlanner::planPath(geometry_msgs::PoseStamped start, geometry_msgs::PoseStamped end, nav_msgs::Path &path)
{
	if (mNavMesh == NULL)
	{
		ROS_WARN("NavMesh is not initialized.");
		return;
	}

	if (mNavMeshQuery == NULL)
	{
		ROS_WARN("NavMeshQuery is not initialized.");
		return;
	}

	int polyCount = 0;
	int straightPathCount = 0;
	dtPolyRef startRef;
	dtPolyRef endRef;
	dtPolyRef polys[MAX_POLYS];
	dtPolyRef straightPathPolys[MAX_POLYS];
	float straightPath[MAX_POLYS*3];
	unsigned char straightPathFlags[MAX_POLYS];
	float spos[3] = { start.pose.position.x, 0.f, start.pose.position.y };
	float epos[3] = { end.pose.position.x, 0.f, end.pose.position.y };
	float polyPickExt[3] = { 2.f, 4.f, 2.f };

	ROS_INFO("Calculating path from (%lf, %lf) to (%lf, %lf)", start.pose.position.x, start.pose.position.y, end.pose.position.x, end.pose.position.y);

	mNavMeshQuery->findNearestPoly(spos, polyPickExt, &mFilter, &startRef, 0);
	mNavMeshQuery->findNearestPoly(epos, polyPickExt, &mFilter, &endRef, 0);

	if (startRef && endRef)
	{
		mNavMeshQuery->findPath(startRef, endRef, spos, epos, &mFilter, polys, &polyCount, MAX_POLYS);
		straightPathCount = 0;
		if (polyCount)
		{
			// In case of partial path, make sure the end point is clamped to the last polygon.
			float tmpEpos[3];
			dtVcopy(tmpEpos, epos);
			if (polys[polyCount-1] != endRef)
				mNavMeshQuery->closestPointOnPoly(polys[polyCount-1], epos, tmpEpos);

			mNavMeshQuery->findStraightPath(spos, tmpEpos, polys, polyCount,
										 straightPath, straightPathFlags,
										 straightPathPolys, &straightPathCount, MAX_POLYS);
		}
	}

	ROS_INFO("PathCount: %d", straightPathCount);
	for (int i = 0; i < straightPathCount; i++)
	{
		geometry_msgs::PoseStamped p;
		p.header.frame_id = "/map";
		p.header.stamp = ros::Time::now();
		p.pose.position.x = straightPath[i*3];
		p.pose.position.y = straightPath[i*3 + 2];
		path.poses.push_back(p);
	}

	path.poses[straightPathCount - 1].pose.orientation = end.pose.orientation;
}

void PathPlanner::goalCB(const geometry_msgs::PoseStamped &goal)
{
	ROS_INFO("Received new goal.");

	if (updateCurrentPosition() == false)
	{
		ROS_ERROR("Failed to update robot position.");
		return;
	}

	if (mLaserScan.get() != NULL)
	{
		// add dynamic obstacles
		uint32_t size = (mLaserScan->angle_max - mLaserScan->angle_min) / mLaserScan->angle_increment;
		for (uint32_t i = 0; i < size; i++)
		{
			if (mLaserScan->ranges[i] >= mLaserScan->range_max)
				continue;

			geometry_msgs::PointStamped tmp, p;
			tmp.header.frame_id = "/base_link";
			tmp.header.stamp = ros::Time(0);
			tmp.point.x = mLaserScan->ranges[i] * cosf(mLaserScan->angle_min + mLaserScan->angle_increment * i);
			tmp.point.y = mLaserScan->ranges[i] * sinf(mLaserScan->angle_min + mLaserScan->angle_increment * i);

			try
			{
				mTransformListener.transformPoint("/map", tmp, p);
			}
			catch (tf::TransformException &ex)
			{
				ROS_ERROR("%s",ex.what());
				return;
			}

			addObstacle(p.point);
		}

		// update the navmesh
		float f = 0.05f;
		if (mTileCache->update(f, mNavMesh) != DT_SUCCESS)
			ROS_ERROR("Something went wrong updating the mesh.");
	}

	nav_msgs::Path path;
	path.header.frame_id = "/map";

	geometry_msgs::PoseStamped start;
	start.pose.position.x = mRobotPosition.getOrigin().getX();
	start.pose.position.y = mRobotPosition.getOrigin().getY();

	planPath(start, goal, path);

	if (mLaserScan.get() != NULL)
		removeAllObstacles();

	ROS_INFO("Size: %d", path.poses.size());
	for (size_t i = 0; i < path.poses.size(); i++)
	{
		std::cout << "Path[" << i << "]: x = " << path.poses[i].pose.position.x << ", y = " << path.poses[i].pose.position.y << std::endl;
	}

	path.header.stamp = ros::Time::now();

	// for rviz
	if (mPathPub.getNumSubscribers())
		mPathPub.publish(path);
}

void PathPlanner::scanCB(const sensor_msgs::LaserScanPtr &scan)
{
	mLaserScan = scan;
}

bool PathPlanner::updateCurrentPosition()
{
	// get current position in the map
	try
	{
		mTransformListener.lookupTransform("/map", "/base_link", ros::Time(0), mRobotPosition);
	}
	catch (tf::TransformException &ex)
	{
		//ROS_ERROR("%s",ex.what());
		return false;
	}

	return true;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "PathPlanner");

	if (argc != 2)
	{
		ROS_ERROR("Invalid input arguments.");
		return 0;
	}

	PathPlanner pp(argv[1]);

	int sleep_rate;
	pp.getNodeHandle()->param<int>("node_sleep_rate", sleep_rate, 50);
	ros::Rate sleep(sleep_rate);

	while (ros::ok())
	{
		sleep.sleep();
		ros::spinOnce();
	}
	return 0;
}
