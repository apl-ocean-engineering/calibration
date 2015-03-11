add_library( lib_camera_calibration 
	board.cpp 
	video.cpp 
	detection.cpp 
	april_tag_board_generator.cpp 
	april_tag_detection_set.cpp  
	file_utils.cpp
        synchronizer.cpp 
	trendnet_time_code.cpp 	)

target_include_directories( lib_camera_calibration 
	PUBLIC ${CMAKE_SOURCE_DIR}/include )
