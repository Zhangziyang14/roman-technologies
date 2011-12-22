/*
 * PathPlanner.cpp
 *
 *  Created on: Nov 14, 2011
 *      Author: hans
 */

#include "logical_unit/PathPlanner.h"

PathPlanner::PathPlanner() :
	mNodeHandle("~"),
	mNavMesh(NULL),
	mNavMeshQuery(NULL)
{
	mNodeHandle.param<std::string>("navmesh", mNavMeshPath, "./navigation/navmesh.bin");
	loadNavmesh();

	mNavMeshQuery = dtAllocNavMeshQuery();
	mNavMeshQuery->init(mNavMesh, 2048);

	// this is used for enabling swimming, climbing and what not ... our robot will not do this :)
	mFilter.setIncludeFlags(0xffff ^ 0x10);
	mFilter.setExcludeFlags(0);

	std::string goalTopic, pathTopic;
	mNodeHandle.param<std::string>("goal_topic", goalTopic, "/move_base_simple/goal");
	mNodeHandle.param<std::string>("path_topic", pathTopic, "/global_path");
	mGoalSub = mNodeHandle.subscribe(goalTopic, 1, &PathPlanner::goalCb, this);
	mPathPub = mNodeHandle.advertise<nav_msgs::Path>(pathTopic, 1);
}

PathPlanner::~PathPlanner()
{
	if (mNavMesh)
		dtFreeNavMesh(mNavMesh);
	if (mNavMeshQuery)
		dtFreeNavMeshQuery(mNavMeshQuery);
}

void PathPlanner::loadNavmesh()
{
	FILE* fp = fopen(mNavMeshPath.c_str(), "rb");
	if (fp == NULL)
	{
		ROS_INFO("Navmesh binary could not be found. Path: %s", mNavMeshPath.c_str());
		return;
	}

	// Read header.
	NavMeshSetHeader header;
	int bytes = fread(&header, sizeof(NavMeshSetHeader), 1, fp);
	if (header.magic != NAVMESHSET_MAGIC)
	{
		ROS_INFO("Magic number does not match");
		fclose(fp);
		return;
	}
	if (header.version != NAVMESHSET_VERSION)
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
	dtStatus status = mNavMesh->init(&header.params);
	if (dtStatusFailed(status))
	{
		ROS_INFO("Parameters init has failed.");
		fclose(fp);
		return;
	}

	// Read tiles.
	for (int i = 0; i < header.numTiles; ++i)
	{
		NavMeshTileHeader tileHeader;
		bytes = fread(&tileHeader, sizeof(tileHeader), 1, fp);
		if (!tileHeader.tileRef || !tileHeader.dataSize)
			break;

		unsigned char* data = (unsigned char*)dtAlloc(tileHeader.dataSize, DT_ALLOC_PERM);
		if (!data) break;
		memset(data, 0, tileHeader.dataSize);
		bytes = fread(data, tileHeader.dataSize, 1, fp);

		mNavMesh->addTile(data, tileHeader.dataSize, DT_TILE_FREE_DATA, tileHeader.tileRef, 0);
	}

	fclose(fp);
}

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
		p.header.frame_id = "/odom";
		p.header.stamp = ros::Time::now();
		p.pose.position.x = straightPath[i*3];
		p.pose.position.y = straightPath[i*3 + 2];
		path.poses.push_back(p);
	}

	path.poses[straightPathCount - 1].pose.orientation = end.pose.orientation;
}

void PathPlanner::goalCb(const geometry_msgs::PoseStamped &goal)
{
	ROS_INFO("Received new goal.");

	nav_msgs::Path path;
	path.header.frame_id = "/odom";

	geometry_msgs::PoseStamped start, end;
	start.pose.position.x = mRobotPosition.getOrigin().getX();
	start.pose.position.y = mRobotPosition.getOrigin().getY();
	end.pose.position.x = goal.pose.position.x;
	end.pose.position.y = goal.pose.position.y;
	end.pose.orientation = goal.pose.orientation;

	planPath(start, end, path);

	ROS_INFO("Size: %d", path.poses.size());
	for (size_t i = 0; i < path.poses.size(); i++)
	{
		path.poses[i].pose.position.x = path.poses[i].pose.position.x;
		path.poses[i].pose.position.y = path.poses[i].pose.position.y;
		std::cout << "Path[" << i << "]: x = " << path.poses[i].pose.position.x << ", y = " << path.poses[i].pose.position.y << std::endl;
	}

	path.header.stamp = ros::Time::now();
	mPathPub.publish(path);
}

void PathPlanner::spin()
{
	tf::TransformListener transformListener;

	while (ros::ok())
	{
		try
		{
			transformListener.lookupTransform("/map", "/base_link", ros::Time(0), mRobotPosition);
		}
		catch (tf::TransformException ex)
		{
			//ROS_ERROR("%s",ex.what());
		}
		ros::spinOnce();
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "PathPlanner");

	PathPlanner pp;
	pp.spin();

	//52.752132 0.000002 61.427223  57.043953 -0.000002 42.184956

	//geometry_msgs::Point p2;
	//p2.x = 57.043953;
	//p2.y = 42.184956;
	return 0;
}