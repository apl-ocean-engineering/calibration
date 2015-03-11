cmake_minimum_required (VERSION 3.1)
project (camera_calibration)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")

find_package( OpenCV REQUIRED )
find_package( GSL REQUIRED )

set( Boost_USE_MULTITHREADED ON )
find_package( Boost REQUIRED COMPONENTS thread system )



include_directories( "/usr/local/include/eigen3" )

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
