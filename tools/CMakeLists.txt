
include_directories( "${CMAKE_SOURCE_DIR}/include" 
	${OpenCV_INCLUDE_DIRS} )

add_executable( align_streams align_streams.cpp )
target_link_libraries( align_streams 
	lib_camera_calibration ${OpenCV_LIBS} 
	${GSL_LIBRARIES}
	${APRILTAGS_LIBS} )
