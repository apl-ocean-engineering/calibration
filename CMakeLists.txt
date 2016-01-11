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

list(APPEND CMAKE_MODULE_PATH "${camera_calibration_SOURCE_DIR}/cmake")

find_package( GTest )
find_package( PCL 1.2 REQUIRED )

option( USE_TBB "Use TBB" ON )
if( USE_TBB )
  add_definitions( -DUSE_TBB -DTBB_USE_DEBUG )
endif()

# set( AplCam_SOURCE_DIR "/home/aaron/workspace/vision_exp/aplcam")
# set( Apriltags_SOURCE_DIR "/home/aaron/workspace/vision_exp/apriltags")

option( USE_APRILTAGS "Use AprilTags" ON )
if( USE_APRILTAGS )
  add_definitions( -DUSE_APRILTAGS )
  find_package( Apriltags REQUIRED )
 	#include_directories( "${apriltags_INCLUDE_DIR}" )
endif()

# Packages with by local Find*.cmake files
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

# Location for test images used by unit and performance tests
set( TEST_DATA_DIR ${CMAKE_SOURCE_DIR}/test/data )
#add_definitions( -DTEST_DATA_DIR="${TEST_DATA_DIR}" )

add_subdirectory (tools)

## Custom Makefile targets for testing cal

set( TEST_CAL_DETECTION_DIR ${CMAKE_CURRENT_BINARY_DIR}/tmp/detections )
set( TEST_CAL_ANNOTATIONS_DIR ${CMAKE_CURRENT_BINARY_DIR}/tmp/annotated )
add_custom_target( cal_detection_dirs
                    mkdir -p ${TEST_CAL_DETECTION_DIR} ${TEST_CAL_ANNOTATIONS_DIR} )

list(APPEND CAL_TEST_CMD ${CMAKE_CURRENT_BINARY_DIR}/tools/cal --detect --board ${TEST_DATA_DIR}/boards/april_poster_45in.yml)
list(APPEND CAL_TEST_CMD --detections-io ${TEST_CAL_DETECTION_DIR} )
list(APPEND CAL_TEST_CMD --draw-detections ${TEST_CAL_ANNOTATIONS_DIR} )
list(APPEND CAL_TEST_CMD ${TEST_DATA_DIR}/images/camhd_apriltags_zoom6_*.png)

add_custom_target( test_cal
                    ${CAL_TEST_CMD}
                    DEPENDS cal cal_detection_dirs )

add_custom_target( gdb_cal
                    gdb --args ${CAL_TEST_CMD}
                    DEPENDS cal cal_detection_dir )
