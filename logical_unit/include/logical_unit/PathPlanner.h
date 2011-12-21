/*
 * PathPlanner.h
 *
 *  Created on: Nov 14, 2011
 *      Author: hans
 */

#ifndef PATHPLANNER_H_
#define PATHPLANNER_H_

#include "ros/ros.h"
#include <fstream>
#include "Detour/DetourNavMesh.h"
#include "Detour/DetourNavMeshQuery.h"
#include "Detour/DetourCommon.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/Point.h"
#include "tf/transform_listener.h"

#define NAVMESHSET_MAGIC ('M'<<24 | 'S'<<16 | 'E'<<8 | 'T') //'MSET'
#define NAVMESHSET_VERSION 1
#define MAX_POLYS 256

struct NavMeshSetHeader
{
	int magic;
	int version;
	int numTiles;
	dtNavMeshParams params;
};

struct NavMeshTileHeader
{
	dtTileRef tileRef;
	int dataSize;
};

class PathPlanner
{
private:
	ros::NodeHandle mNodeHandle;
	ros::Subscriber mGoalSub;
	ros::Publisher mPathPub;
	std::string mNavMeshPath;
	dtNavMesh *mNavMesh;
	dtNavMeshQuery *mNavMeshQuery;
	dtQueryFilter mFilter;
	tf::StampedTransform mRobotPosition;

	void loadNavmesh();
public:
	PathPlanner();
	~PathPlanner();

	void spin();

	void planPath(geometry_msgs::PoseStamped start, geometry_msgs::PoseStamped end, nav_msgs::Path &path);

	void goalCb(const geometry_msgs::PoseStamped &goal);
};

#endif /* PATHPLANNER_H_ */
