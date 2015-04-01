
include_directories( "${CMAKE_SOURCE_DIR}/include"  )

link_libraries(	lib_camera_calibration ${OpenCV_LIBS} 
	${GSL_LIBRARIES}
	${Boost_LIBRARIES}
        ${CERES_LIBRARIES}
	${KyotoCabinet_LIBRARIES}
        cryptopp )

if( USE_APRILTAGS )
  link_libraries( ${APRILTAGS_LIBS} )
endif()



add_executable( extract extract.cpp )

add_executable( build_detection_db build_detection_db.cpp )
if( USE_TBB )
  target_link_libraries( build_detection_db tbb )
endif()



add_executable( calculate_calibration_errors calculate_calibration_errors.cpp )

add_executable( align_streams align_streams.cpp )

add_executable( composite_video composite_video.cpp )

add_executable( calibration calibration.cpp )

add_executable( calibration_video calibration_video.cpp )

add_executable( stereo_calibration stereo_calibration.cpp )
target_compile_definitions( stereo_calibration PUBLIC ${PCL_DEFINITIONS})
target_include_directories( stereo_calibration PUBLIC ${PCL_INCLUDE_DIRS} )
target_link_libraries( stereo_calibration ${PCL_LIBRARIES} )
