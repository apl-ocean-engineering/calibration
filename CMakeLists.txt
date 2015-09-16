cmake_minimum_required (VERSION 2.8)
project (camera_calibration)

set(CMAKE_C_FLAGS "-ggdb -Wall")
set(CMAKE_CXX_FLAGS "-ggdb -Wall")

#set(CMAKE_BUILD_TYPE Debug )
#set(CMAKE_C_FLAGS_DEBUG, "-Wall -ggdb")
#set(CMAKE_CXX_FLAGS_DEBUG, "-Wall -ggdb")
#
#set(CMAKE_C_FLAGS_RELEASE, "-Werror")
#set(CMAKE_CXX_FLAGS_RELEASE, "-Werror")

find_package( gtest )
find_package( PCL 1.2 REQUIRED )

option( USE_TBB "Use TBB" ON )
if( USE_APRILTAGS )
  add_definitions( -DUSE_APRILTAGS )
 	#include_directories( "${apriltags_INCLUDE_DIR}" )
endif()

option( USE_APRILTAGS "Use AprilTags" ON )
if( USE_TBB )
  add_definitions( -DUSE_TBB -DTBB_USE_DEBUG )
endif()


# Packages with by local Find*.cmake files
set( AplCam_SOURCE_DIR "/home/aaron/workspace/vision_exp/aplcam")
list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake" "${AplCam_SOURCE_DIR}/cmake")
find_package( AplCam REQUIRED )

## Get these from aplcam's cmake directory
#find_package( TCLAP REQUIRED )
#find_package( Eigen REQUIRED )

# KyotoCabinet stuff
find_path(KyotoCabinet_INCLUDE_DIR kcdb.h
  HINTS ${PC_KyotoCabinet_INCLUDEDIR} ${PC_KyotoCabinet_INCLUDE_DIRS})

find_library(KyotoCabinet_LIBRARY NAMES libkyotocabinet kyotocabinet
  HINTS ${PC_KyotoCabinet_LIBDIR} ${PC_KyotoCabinet_LIBRARY_DIRS})

set(KyotoCabinet_LIBRARIES ${KyotoCabinet_LIBRARY})
set(KyotoCabinet_INCLUDE_DIRS ${KyotoCabinet_INCLUDE_DIR})

add_definitions( -std=c++11 )

set( camera_calibration_INCLUDE_DIRS
	   ${CMAKE_SOURCE_DIR}/include
	   ${aplcam_INCLUDE_DIRS}
      ${KyotoCabinet_INCLUDE_DIRS} )

set( camera_calibration_LIBS
	    ${aplcam_LIBS}
      ${KyotoCabinet_LIBRARIES} )

link_directories( ${PCL_LIBRARY_DIRS} )


add_subdirectory (tools)
