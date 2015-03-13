
include_directories( "${CMAKE_SOURCE_DIR}/include" 
	${OpenCV_INCLUDE_DIRS} )
link_libraries(	lib_camera_calibration ${OpenCV_LIBS} 
	${GSL_LIBRARIES}
	${Boost_LIBRARIES}
	${APRILTAGS_LIBS} )

add_executable( extract extract.cpp )

add_executable( align_streams align_streams.cpp )

add_executable( calibration calibration.cpp )
target_link_libraries( calibration cryptopp )

add_executable( stereo_calibration stereo_calibration.cpp )
