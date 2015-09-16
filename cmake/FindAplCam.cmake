

if (NOT AplCam_SOURCE_DIR)
	message( STATUS "Using default value for AplCam_SOURCE_DIR" )
	set( AplCam_SOURCE_DIR "/home/aaron/workspace/vision_exp/aplcam")
endif ()

## Try pulling in the aplcam as an external project
include( ExternalProject )
ExternalProject_Add( aplcam
  DOWNLOAD_COMMAND ""
  SOURCE_DIR ${AplCam_SOURCE_DIR}
  PREFIX ${CMAKE_CURRENT_BINARY_DIR}/aplcam
  CMAKE_ARGS "-DCMAKE_INSTALL_PREFIX=${INSTALL_DIR}"
  BUILD_COMMAND ${CMAKE_COMMAND} --build . -- aplcam
  INSTALL_COMMAND ""
)

ExternalProject_Add_Step( aplcam forceconfigure
     COMMAND ${CMAKE_COMMAND} -E echo "Force build of aplcam "
     DEPENDEES configure
     DEPENDERS build
     ALWAYS 1)

if( USE_APRILTAGS )
  find_package( Apriltags REQUIRED )
endif()

## AplCam has external dependencies
find_package( GSL REQUIRED )
find_package( OpenCV REQUIRED )
find_package( Ceres REQUIRED )
find_package( Boost REQUIRED COMPONENTS thread system filesystem )
set( Boost_USE_MULTITHREADED ON )

ExternalProject_Get_Property( aplcam SOURCE_DIR BINARY_DIR )
add_library( libaplcam STATIC IMPORTED )
set_property( TARGET libaplcam PROPERTY IMPORTED_LOCATION ${BINARY_DIR}/lib/libaplcam.a )

set( aplcam_INCLUDE_DIRS
     ${SOURCE_DIR}/include
     ${apriltags_INCLUDE_DIRS}
     ${OpenCV_INCLUDE_DIRS}
		 ${EIGEN_INCLUDE_DIR}
		 ${GSL_INCLUDE_DIRS}
			${CERES_INCLUDE_DIRS} )

set( aplcam_LIBS
     libaplcam
     libapriltags
     cryptopp
     ${OpenCV_LIBS}
     ${GSL_LIBRARIES}
     ${Boost_LIBRARIES}
 		 ${CERES_LIBRARIES}
		 gtest )

find_package_handle_standard_args( aplcam DEFAULT_MSG aplcam_INCLUDE_DIRS aplcam_LIBS )
