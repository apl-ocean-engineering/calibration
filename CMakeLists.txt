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

find_package( OpenCV REQUIRED )
find_package( PCL 1.2 REQUIRED )
find_package( Ceres REQUIRED )

# Packages supported by local Find*.cmake files
list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")
find_package( GSL REQUIRED )
find_package( Eigen REQUIRED )



# KyotoCabinet stuff
find_path(KyotoCabinet_INCLUDE_DIR kcdb.h
  HINTS ${PC_KyotoCabinet_INCLUDEDIR} ${PC_KyotoCabinet_INCLUDE_DIRS})

find_library(KyotoCabinet_LIBRARY NAMES libkyotocabinet kyotocabinet
  HINTS ${PC_KyotoCabinet_LIBDIR} ${PC_KyotoCabinet_LIBRARY_DIRS})

set(KyotoCabinet_LIBRARIES ${KyotoCabinet_LIBRARY})
set(KyotoCabinet_INCLUDE_DIRS ${KyotoCabinet_INCLUDE_DIR})

find_package( Boost REQUIRED COMPONENTS thread system filesystem )
set( Boost_USE_MULTITHREADED ON )

include_directories( ${EIGEN_INCLUDE_DIR} 
	${OpenCV_INCLUDE_DIRS} 
        ${CERES_INCLUDE_DIRS} 
        ${KyotoCabinet_INCLUDE_DIRS} )

link_directories( ${PCL_LIBRARY_DIRS} )

add_definitions( -std=c++11 )

add_subdirectory(../aplcam "${CMAKE_BINARY_DIR}/aplcam")

if( USE_APRILTAGS )
	add_definitions( -DUSE_APRILTAGS )
	include_directories( "${APRILTAGS_INCLUDE_DIR}" )
endif()

if( USE_TBB )
  add_definitions( -DUSE_TBB -DTBB_USE_DEBUG )
endif()


add_subdirectory (tools)

