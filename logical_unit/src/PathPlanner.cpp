/*
 * PathPlanner.cpp
 *
 *  Created on: Nov 14, 2011
 *      Author: hans
 */

#include "PathPlanner.h"

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

void PathPlanner::planPath(geometry_msgs::Point start, geometry_msgs::Point end, std::vector<geometry_msgs::Point> &path)
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
	float spos[3] = { start.x, 0.f, start.y };
	float epos[3] = { end.x, 0.f, end.y };
	float polyPickExt[3] = { 2.f, 4.f, 2.f };

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

	for (int i = 0; i < straightPathCount; i++)
	{
		geometry_msgs::Point p;
		p.x = straightPath[i*3];
		p.y = straightPath[i*3 + 2];
		path.push_back(p);
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "PathPlanner");

	PathPlanner pp;

	geometry_msgs::Point p1;
	geometry_msgs::Point p2;

	//52.752132 0.000002 61.427223  57.043953 -0.000002 42.184956
	p1.x = 52.752132;
	p1.y = 61.427223;

	p2.x = 57.043953;
	p2.y = 42.184956;

	std::vector<geometry_msgs::Point> path;
	pp.planPath(p1, p2, path);

	for (size_t i = 0; i < path.size(); i++)
		std::cout << "Path[" << i << "]: x = " << path[i].x << ", y = " << path[i].y << std::endl;
	return 0;
}
