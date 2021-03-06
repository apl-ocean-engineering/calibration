


# set( SONAR_SRCS
#   ../lib/sonar_calibration_solver.cpp )


#fips_include_directories( ${camera_calibration_INCLUDE_DIRS} )

# set( CALIBRATION_SRCS
#     ../lib/input_queue.cpp
#     ../lib/detection_io.cpp )
# #   ../lib/calibrator.cpp
# #   ../lib/calibration_opts.cpp )
# #
# # add_executable( calibration calibration.cpp
# #   ${CALIBRATION_SRCS} )

fips_begin_app( cal cmdline )
  fips_files( cal.cpp ) #cal_impl.cpp ${CALIBRATION_SRCS} )
  fips_src( subcommands/ )

  fips_deps( camera_calibration g3logger activeobject )
fips_end_app()

# add_executable( extract extract.cpp )
#
# add_executable( build_stereo_detection_db build_stereo_detection_db.cpp )
#
# add_executable( build_detection_db build_detection_db.cpp )
# if( USE_TBB )
#   target_link_libraries( build_detection_db tbb )
# endif()
#
# add_executable( simulate_detection_db simulate_detection_db.cpp )
#
#
# add_executable( calculate_calibration_errors calculate_calibration_errors.cpp )
#
# add_executable( align_streams align_streams.cpp )
#
# add_executable( test_video test_video.cpp )


# add_executable( video_calibration video_calibration.cpp  ${CALIBRATION_SRCS} )
#
# add_executable( video_calibration_permutations
#   video_calibration_permutations.cpp
#   ${CALIBRATION_SRCS} )
#
# add_executable( video_calibration_permutations_new
#   video_calibration_permutations_new.cpp
#   ${CALIBRATION_SRCS} )
#
#
# add_executable( calibration_db_reproject calibration_db_reproject.cpp ${CALIBRATION_SRCS} )
# if( USE_TBB )
#   target_link_libraries( calibration_db_reproject tbb )
# endif()
#
#
# #add_executable( calibration_reproject calibration_reproject.cpp ${CALIBRATION_SRCS} )
#
# add_executable( calibration_db_dump calibration_db_dump.cpp )
#
# add_executable( db_stereo_calibration db_stereo_calibration.cpp )
#
# #add_executable( composite_video  ${STEREO_PROCESSOR_SRCS} )
#
# add_executable( image_processor image_processor.cpp )
#
# add_executable( sonar_calibration sonar_calibration.cpp
#   ${SONAR_SRCS} )
#
# add_executable( annotate_sphere_locations annotate_sphere_locations.cpp )
#
# add_executable( stereo_calibration stereo_calibration.cpp )
# target_compile_definitions( stereo_calibration PUBLIC ${PCL_DEFINITIONS})
# target_include_directories( stereo_calibration PUBLIC ${PCL_INCLUDE_DIRS} )
# target_link_libraries( stereo_calibration ${PCL_LIBRARIES} )
#
# add_executable( validate_sonar validate_sonar.cpp
#   ${SONAR_SRCS}
#   ../lib/sonar_image_warper.cpp
#   ../lib/sonar_detections.cpp )
#
#
#
# set( VISUALIZE_SRCS
# ../lib/visualize_app_common.cpp
# ../lib/sonar_image_warper.cpp
# ../lib/sonar_detections.cpp
# ../lib/background_segmenter.cpp
# )
#
# add_executable( visualize_pc MACOSX_BUNDLE visualize_pc.cpp
#   ${SONAR_SRCS}
#   ${VISUALIZE_SRCS}
#  )
# target_compile_definitions( visualize_pc PUBLIC ${PCL_DEFINITIONS})
# target_include_directories( visualize_pc PUBLIC ${PCL_INCLUDE_DIRS} )
# target_link_libraries( visualize_pc ${PCL_LIBRARIES} )
#
# add_executable( visualize_img visualize_img.cpp
#   ../lib/graphcut.cpp
#   ../lib/rm_graphcut.cpp
#   ../lib/gmm.cpp
#   ../lib/dark_channel.cpp
#   ${SONAR_SRCS}
#   ${VISUALIZE_SRCS} )
# target_compile_definitions( visualize_img PUBLIC ${PCL_DEFINITIONS})
# target_include_directories( visualize_img PUBLIC ${PCL_INCLUDE_DIRS} )
# target_link_libraries( visualize_img ${PCL_LIBRARIES} )
#
#
#
# add_executable( deinterlace deinterlace.cpp )
#
# add_executable( extract_background_image extract_background_image.cpp )
