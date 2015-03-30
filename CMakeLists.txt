cmake_minimum_required (VERSION 3.1)
project (camera_calibration)
list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")


set(MAKE_C_FLAGS, "-Wall")
set(MAKE_CXX_FLAGS, "-Wall")

set(MAKE_C_FLAGS_RELEASE, "-Werror")
set(MAKE_CXX_FLAGS_RELEASE, "-Werror")

set( EIGEN_INCLUDE_DIR "/usr/local/include/eigen3/" )

find_package( OpenCV REQUIRED )
find_package( PCL 1.2 REQUIRED )
find_package( GSL REQUIRED )
find_package( Ceres REQUIRED )

find_path(KyotoCabinet_INCLUDE_DIR kcdb.h
  HINTS ${PC_KyotoCabinet_INCLUDEDIR} ${PC_KyotoCabinet_INCLUDE_DIRS})

find_library(KyotoCabinet_LIBRARY NAMES libkyotocabinet kyotocabinet
  HINTS ${PC_KyotoCabinet_LIBDIR} ${PC_KyotoCabinet_LIBRARY_DIRS})

set(KyotoCabinet_LIBRARIES ${KyotoCabinet_LIBRARY})
set(KyotoCabinet_INCLUDE_DIRS ${KyotoCabinet_INCLUDE_DIR})


set( Boost_USE_MULTITHREADED ON )
find_package( Boost REQUIRED COMPONENTS thread system filesystem )



include_directories( "/usr/local/include/eigen3" )

link_directories( ${PCL_LIBRARY_DIRS} )

option( USE_APRILTAGS "Use AprilTags" ON )

if( USE_APRILTAGS )
	add_definitions( -DUSE_APRILTAGS )
	find_path( APRILTAGS_INCLUDE_DIR AprilTags/TagDetection.h
		PATHS ${CMAKE_SOURCE_DIR}/third_party/apriltags/build/include  )
	set( APRILTAGS_LIBS ${CMAKE_SOURCE_DIR}/third_party/apriltags/build/lib/libapriltags.a )

	include_directories( "${APRILTAGS_INCLUDE_DIR}" )
endif()

add_subdirectory (tools)
add_subdirectory (lib)
