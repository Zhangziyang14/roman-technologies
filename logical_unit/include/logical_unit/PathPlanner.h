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
#include "DetourNavMesh.h"
#include "DetourNavMeshBuilder.h"
#include "DetourNavMeshQuery.h"
#include "DetourCommon.h"
#include "DetourTileCache.h"
#include "DetourTileCacheBuilder.h"
#include "logical_unit/fastlz.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/Point.h"
#include "tf/transform_listener.h"
#include "sensor_msgs/LaserScan.h"

#define TILECACHESET_MAGIC ('T'<<24 | 'S'<<16 | 'E'<<8 | 'T') //'TSET'
#define TILECACHESET_VERSION 1
#define MAX_POLYS 256

#define ROBOT_RADIUS 0.5
#define MAX_OBSTACLES 360

/// These are just sample areas to use consistent values across the samples.
/// The use should specify these base on his needs.
enum SamplePolyAreas
{
	SAMPLE_POLYAREA_GROUND,
	SAMPLE_POLYAREA_WATER,
	SAMPLE_POLYAREA_ROAD,
	SAMPLE_POLYAREA_DOOR,
	SAMPLE_POLYAREA_GRASS,
	SAMPLE_POLYAREA_JUMP,
};
enum SamplePolyFlags
{
	SAMPLE_POLYFLAGS_WALK		= 0x01,		// Ability to walk (ground, grass, road)
	SAMPLE_POLYFLAGS_SWIM		= 0x02,		// Ability to swim (water).
	SAMPLE_POLYFLAGS_DOOR		= 0x04,		// Ability to move through doors.
	SAMPLE_POLYFLAGS_JUMP		= 0x08,		// Ability to jump.
	SAMPLE_POLYFLAGS_DISABLED	= 0x10,		// Disabled polygon
	SAMPLE_POLYFLAGS_ALL		= 0xffff	// All abilities.
};

struct TileCacheSetHeader
{
	int magic;
	int version;
	int numTiles;
	dtNavMeshParams meshParams;
	dtTileCacheParams cacheParams;
};

struct TileCacheTileHeader
{
	dtCompressedTileRef tileRef;
	int dataSize;
};

struct FastLZCompressor : public dtTileCacheCompressor
{
	virtual int maxCompressedSize(const int bufferSize)
	{
		return (int)(bufferSize* 1.05f);
	}

	virtual dtStatus compress(const unsigned char* buffer, const int bufferSize,
							  unsigned char* compressed, const int /*maxCompressedSize*/, int* compressedSize)
	{
		*compressedSize = fastlz_compress((const void *const)buffer, bufferSize, compressed);
		return DT_SUCCESS;
	}

	virtual dtStatus decompress(const unsigned char* compressed, const int compressedSize,
								unsigned char* buffer, const int maxBufferSize, int* bufferSize)
	{
		*bufferSize = fastlz_decompress(compressed, compressedSize, buffer, maxBufferSize);
		return *bufferSize < 0 ? DT_FAILURE : DT_SUCCESS;
	}
};

struct LinearAllocator : public dtTileCacheAlloc
{
	unsigned char* buffer;
	int capacity;
	int top;
	int high;

	LinearAllocator(const int cap) : buffer(0), capacity(0), top(0), high(0)
	{
		resize(cap);
	}

	~LinearAllocator()
	{
		dtFree(buffer);
	}

	void resize(const int cap)
	{
		if (buffer) dtFree(buffer);
		buffer = (unsigned char*)dtAlloc(cap, DT_ALLOC_PERM);
		capacity = cap;
	}

	virtual void reset()
	{
		high = dtMax(high, top);
		top = 0;
	}

	virtual void* alloc(const int size)
	{
		if (!buffer)
			return 0;
		if (top+size > capacity)
			return 0;
		unsigned char* mem = &buffer[top];
		top += size;
		return mem;
	}

	virtual void free(void* /*ptr*/)
	{
		// Empty
	}
};

struct MeshProcess : public dtTileCacheMeshProcess
{
	inline MeshProcess()
	{
	}

	virtual void process(struct dtNavMeshCreateParams* params,
						 unsigned char* polyAreas, unsigned short* polyFlags)
	{
		// Update poly flags from areas.
		for (int i = 0; i < params->polyCount; ++i)
		{
			if (polyAreas[i] == DT_TILECACHE_WALKABLE_AREA)
				polyAreas[i] = SAMPLE_POLYAREA_GROUND;

			if (polyAreas[i] == SAMPLE_POLYAREA_GROUND ||
				polyAreas[i] == SAMPLE_POLYAREA_GRASS ||
				polyAreas[i] == SAMPLE_POLYAREA_ROAD)
			{
				polyFlags[i] = SAMPLE_POLYFLAGS_WALK;
			}
			else if (polyAreas[i] == SAMPLE_POLYAREA_WATER)
			{
				polyFlags[i] = SAMPLE_POLYFLAGS_SWIM;
			}
			else if (polyAreas[i] == SAMPLE_POLYAREA_DOOR)
			{
				polyFlags[i] = SAMPLE_POLYFLAGS_WALK | SAMPLE_POLYFLAGS_DOOR;
			}
		}
	}
};

/// Handles incoming goal requests, sends out paths to follow.
class PathPlanner
{
private:
	ros::NodeHandle mNodeHandle;				/// This nodes nodehandle
	ros::Subscriber mGoalSub;
	ros::Publisher mPathPub;					/// Publishes paths to received goals
	ros::Subscriber mScanSub;
	std::string mTileCachePath;					/// Path to the tilecache .bin file
	tf::StampedTransform mRobotPosition;

	dtNavMesh *mNavMesh;						/// Navmesh object
	dtNavMeshQuery *mNavMeshQuery;				/// Query handler for the navmesh
	dtTileCache *mTileCache;					/// Handles dynamic obstacles
	struct LinearAllocator *mTalloc;
	struct FastLZCompressor *mTComp;
	struct MeshProcess *mTmpProc;
	dtQueryFilter mFilter;						/// Not currently used

	tf::TransformListener mTransformListener;	/// Listens to current robot position
	sensor_msgs::LaserScanPtr mLaserScan;

	void loadNavmesh();
public:
	PathPlanner(const char *navmesh_file);
	~PathPlanner();

	void spin();
	void planPath(geometry_msgs::PoseStamped start, geometry_msgs::PoseStamped end, nav_msgs::Path &path);
	void goalCB(const geometry_msgs::PoseStamped &goal);
	void scanCB(const sensor_msgs::LaserScanPtr &scan);
	bool updateCurrentPosition();

	inline ros::NodeHandle* getNodeHandle() { return &mNodeHandle; };

	void addObstacle(geometry_msgs::Point p);
	void removeAllObstacles();
};

#endif /* PATHPLANNER_H_ */
