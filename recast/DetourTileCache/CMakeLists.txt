CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

SET(detourtilecache_SRCS 
	Source/DetourTileCache.cpp
	Source/DetourTileCacheBuilder.cpp
)

SET(detourtilecache_HDRS
	Include/DetourTileCache.h
	Include/DetourTileCacheBuilder.h
)

INCLUDE_DIRECTORIES(Include 
	../Detour/Include
	../Recast/Include
)

rosbuild_add_library(DetourTileCache ${detourtilecache_SRCS} ${detourtilecache_HDRS})
