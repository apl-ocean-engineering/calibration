
APPS := calibration calibration_artificial calibration_april \
  extract_frame trendnet_time_code_test watch_for_change extract_static_frames \
  align_streams

all: $(APPS)

APRILTAGS_PATH = third_party/apriltags/build
FFTS_PATH = third_party/ffts/build

CXXFLAGS =  -g -I/opt/opencv/include -I$(APRILTAGS_PATH)/include -I/usr/local/include/eigen3  -DUSE_APRILTAGS \
	    -I$(FFTS_PATH)/include/ffts 
LDFLAGS = $(CXXFLAGS) -L/opt/opencv/lib -L$(APRILTAGS_PATH)/lib -lgsl

	  #-L$(FFTS)/lib -lffts

LIBS = -lapriltags -lopencv_core -lopencv_calib3d -lopencv_features2d -lopencv_highgui -lopencv_imgproc  -lgsl
LD = g++


extract_frame: extract_frame.o
	$(LD) -o $@ $(LDFLAGS) $(LIBS) $^

align_streams: align_streams.o trendnet_time_code.o file_utils.o 
	$(LD) -o $@ $(LDFLAGS) $(LIBS) $^ 

extract_static_frames: extract_static_frames.o trendnet_time_code.o
	$(LD) -o $@ $(LDFLAGS) $(LIBS) $^

calibration: calibration.o file_utils.o board.o detection.o
	$(LD) -o $@ $(LDFLAGS) $(LIBS) $^

trendnet_time_code_test: trendnet_time_code_test.o trendnet_time_code.o
	$(LD) -o $@ $(LDFLAGS) $(LIBS) $^

watch_for_change: watch_for_change.o trendnet_time_code.o
	$(LD) -o $@ $(LDFLAGS) $(LIBS) $^


calibration_artificial: calibration_artificial.o chess_board_generator.o
	$(LD) -o $@ $(LDFLAGS) $(LIBS) $^

calibration_april: calibration_april.o april_tag_board_generator.o april_tag_detection_set.o
	$(LD) -o $@ $(LDFLAGS) $(LIBS) $^


.cpp.o:
	$(CXX) -c -o $@ $(CXXFLAGS) $<


clean:
	rm -f $(APPS)
	rm -f *.o
