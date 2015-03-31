
include_directories( "${CMAKE_SOURCE_DIR}/include"  )

link_libraries(	lib_camera_calibration ${OpenCV_LIBS} 
        ${PCL_LIBRARIES}
	${GSL_LIBRARIES}
	${Boost_LIBRARIES}
	${APRILTAGS_LIBS}
        ${CERES_LIBRARIES}
	${KyotoCabinet_LIBRARIES}
        cryptopp )

add_definitions(${PCL_DEFINITIONS})

add_executable( extract extract.cpp )

add_executable( build_detection_db build_detection_db.cpp )

add_executable( calculate_calibration_errors calculate_calibration_errors.cpp )

add_executable( align_streams align_streams.cpp )

add_executable( composite_video composite_video.cpp )

add_executable( calibration calibration.cpp )

add_executable( calibration_video calibration_video.cpp )

add_executable( stereo_calibration stereo_calibration.cpp )
