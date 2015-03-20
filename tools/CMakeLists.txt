
include_directories( "${CMAKE_SOURCE_DIR}/include" 
	${OpenCV_INCLUDE_DIRS} 
        ${PCL_INCLUDE_DIRS}
        ${CERES_INCLUDE_DIRS} )

link_libraries(	lib_camera_calibration ${OpenCV_LIBS} 
        ${PCL_LIBRARIES}
	${GSL_LIBRARIES}
	${Boost_LIBRARIES}
	${APRILTAGS_LIBS}
        ${CERES_LIBRARIES}
        cryptopp )

add_definitions(${PCL_DEFINITIONS})

add_executable( extract extract.cpp )

add_executable( align_streams align_streams.cpp )

add_executable( calibration calibration.cpp )

add_executable( stereo_calibration stereo_calibration.cpp )
