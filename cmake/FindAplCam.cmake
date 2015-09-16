
## This is intended to be as minimal as possible, it finds the aplcam source
# and defines the ExternalProject.  Then chains to aplcam/cmake/AplCamConfig.cmake
# for further configuration

include( ExternalProject )

if (AplCam_SOURCE_DIR)
	message( STATUS "Using aplcam from ${AplCam_SOURCE_DIR}" )

	ExternalProject_Add( aplcam
	  DOWNLOAD_COMMAND ""
	  SOURCE_DIR ${AplCam_SOURCE_DIR}
	  PREFIX ${CMAKE_CURRENT_BINARY_DIR}/aplcam
	  CMAKE_ARGS "-DCMAKE_INSTALL_PREFIX=${INSTALL_DIR}"
	  BUILD_COMMAND ${CMAKE_COMMAND} --build . -- aplcam
	  INSTALL_COMMAND ""
	)

	## If you've pulled from source, always try to rebuild the source
	ExternalProject_Add_Step( aplcam forceconfigure
	     COMMAND ${CMAKE_COMMAND} -E echo "Force build of aplcam "
	     DEPENDEES configure
	     DEPENDERS build
	     ALWAYS 1)

else ()
	message( STATUS "Pulling AplCam from git...")
	message( FATAL_ERROR "Except that's not set up yet....")
endif ()

ExternalProject_Get_Property( aplcam SOURCE_DIR )
set( aplcam_SOURCE_DIR ${SOURCE_DIR} )

include( ${aplcam_SOURCE_DIR}/cmake/AplCamConfig.cmake )
