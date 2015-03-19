add_library( lib_camera_calibration 
	board.cpp 
        image.cpp
	video.cpp 
	detection.cpp 
        my_undistort.cpp
	april_tag_board_generator.cpp 
	april_tag_detection_set.cpp  
	file_utils.cpp
        synchronizer.cpp 
	trendnet_time_code.cpp 
        camera_model.cpp
        pinhole_camera.cpp
        distortion_model.cpp
        angular_polynomial.cpp
        radial_polynomial.cpp
        camera_factory.cpp )
      # fisheye.cpp

target_include_directories( lib_camera_calibration 
	PUBLIC ${CMAKE_SOURCE_DIR}/include )
