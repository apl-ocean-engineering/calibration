if (NOT FIPS_IMPORT)
  cmake_minimum_required(VERSION 3.5.0)

  get_filename_component(FIPS_ROOT_DIR "../fips" ABSOLUTE)
  include("${FIPS_ROOT_DIR}/cmake/fips.cmake")

  fips_setup(PROJECT camera_calibration)
endif()

set( CUDA_USE_STATIC_CUDA_RUNTIME OFF )
find_package( OpenCV REQUIRED core highgui imgproc )

#find_package( PCL 1.2 REQUIRED )

option( USE_TBB "Use TBB" ON )
if( USE_TBB )
  add_definitions( -DUSE_TBB -DTBB_USE_DEBUG )
endif()

find_package( Ceres REQUIRED )

find_package( GSL REQUIRED )
find_package( Boost REQUIRED COMPONENTS thread system filesystem )
set( Boost_USE_MULTITHREADED ON )

# KyotoCabinet stuff
find_path(KyotoCabinet_INCLUDE_DIR kcdb.h
  HINTS ${PC_KyotoCabinet_INCLUDEDIR} ${PC_KyotoCabinet_INCLUDE_DIRS})

find_library(KyotoCabinet_LIBRARY NAMES libkyotocabinet kyotocabinet
  HINTS ${PC_KyotoCabinet_LIBDIR} ${PC_KyotoCabinet_LIBRARY_DIRS})

set(KyotoCabinet_LIBRARIES ${KyotoCabinet_LIBRARY})
set(KyotoCabinet_INCLUDE_DIRS ${KyotoCabinet_INCLUDE_DIR})

add_definitions( -std=c++11 )

# set( camera_calibration_INCLUDE_DIRS
# 	   ${CMAKE_SOURCE_DIR}/include
#      ${KyotoCabinet_INCLUDE_DIRS} )

# set( camera_calibration_LIBS
#       ${OpenCV_LIBS}
#       ${Boost_LIBRARIES}
#   		${CERES_LIBRARY}
#   		${GSL_LIBRARIES}
#       ${KyotoCabinet_LIBRARIES}
#     glog
#   cryptopp )

# link_directories( ${PCL_LIBRARY_DIRS} )

# Location for test images used by unit and performance tests
set( TEST_DATA_DIR ${CMAKE_SOURCE_DIR}/test/data )
#add_definitions( -DTEST_DATA_DIR="${TEST_DATA_DIR}" )

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

fips_include_directories( include/ )

fips_add_subdirectory( lib/ )

if( NOT FIPS_IMPORT )
  fips_add_subdirectory( tools/ )

  # if( FIPS_UNITTESTS )
  #   ## Build a local copy of gtest (as per the GTest FAQ)
  #   fips_add_subdirectory( test/unit/ )
  # endif()

  fips_finish()
endif()
