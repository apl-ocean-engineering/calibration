cmake_minimum_required (VERSION 2.8)
project (camera_calibration)

set(MAKE_C_FLAGS, "-Wall -ggdb")
set(MAKE_CXX_FLAGS, "-Wall -ggdb")

set(MAKE_C_FLAGS_RELEASE, "-Werror")
set(MAKE_CXX_FLAGS_RELEASE, "-Werror")

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

option( USE_TBB "Use TBB" ON )
if( USE_TBB )
  add_definitions( -DUSE_TBB -DTBB_USE_DEBUG=1 )
endif()

option( USE_APRILTAGS "Use AprilTags" ON )
if( USE_APRILTAGS )
	add_definitions( -DUSE_APRILTAGS )
        add_subdirectory (third_party/apriltags)

        find_path( APRILTAGS_INCLUDE_DIR AprilTags/TagDetection.h
          PATHS ${CMAKE_SOURCE_DIR}/third_party/apriltags/  )
        set( APRILTAGS_LIBS ${CMAKE_BINARY_DIR}/third_party/apriltags/libapriltags.a )


	include_directories( "${APRILTAGS_INCLUDE_DIR}" )

endif()

add_subdirectory (tools)
add_subdirectory (lib)

