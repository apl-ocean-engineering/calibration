
include_directories( "${CMAKE_SOURCE_DIR}/include"  )
include_directories( ${aplcam_INCLUDE_DIR} 
	             ${tclap_INCLUDE_DIR} )

link_libraries(	${aplcam_LIBS}
        ${CERES_LIBRARIES}
	${KyotoCabinet_LIBRARIES}
	${GSL_LIBRARIES}
	${Boost_LIBRARIES}
        cryptopp )

if( USE_APRILTAGS )
  link_libraries( ${APRILTAGS_LIBS} )
endif()

link_libraries( ${OpenCV_LIBS} )


add_executable( extract extract.cpp )

add_executable( build_stereo_detection_db build_stereo_detection_db.cpp )

add_executable( build_detection_db build_detection_db.cpp )
if( USE_TBB )
  target_link_libraries( build_detection_db tbb )
endif()

add_executable( simulate_detection_db simulate_detection_db.cpp )


add_executable( calculate_calibration_errors calculate_calibration_errors.cpp )

add_executable( align_streams align_streams.cpp )

add_executable( composite_video composite_video.cpp )

add_executable( calibration calibration.cpp )

add_executable( video_calibration video_calibration.cpp ../lib/calibrator.cpp )

add_executable( video_calibration_permutations video_calibration_permutations.cpp ../lib/calibrator.cpp )
if( USE_TBB )
  target_link_libraries( video_calibration_permutations tbb )
endif()

add_executable( calibration_reproject calibration_reproject.cpp )
if( USE_TBB )
  target_link_libraries( calibration_reproject tbb )
endif()

add_executable( calibration_db_dump calibration_db_dump.cpp )

add_executable( db_stereo_calibration db_stereo_calibration.cpp )

add_executable( stereo_processor stereo_processor.cpp )

add_executable( stereo_calibration stereo_calibration.cpp )
target_compile_definitions( stereo_calibration PUBLIC ${PCL_DEFINITIONS})
target_include_directories( stereo_calibration PUBLIC ${PCL_INCLUDE_DIRS} )
target_link_libraries( stereo_calibration ${PCL_LIBRARIES} )


add_executable( deinterlace deinterlace.cpp )
target_include_directories( deinterlace 
	PUBLIC ${tclap_INCLUDE_DIR} )
