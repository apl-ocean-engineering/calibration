
APPS := calibration calibration_artificial calibration_april

all: $(APPS)

APRILTAGS = ../apriltags/build

CXXFLAGS = -I/opt/opencv/include -I$(APRILTAGS)/include -I/usr/local/include/eigen3
LDFLAGS = $(CXXFLAGS) -L/opt/opencv/lib -L$(APRILTAGS)/lib
LIBS = -lopencv_core -lopencv_calib3d -lopencv_features2d -lopencv_highgui -lopencv_imgproc
LD = g++


calibration: calibration.o
	$(LD) -o $@ $(LDFLAGS) $< $(LIBS)

calibration_artificial: calibration_artificial.o chess_board_generator.o
	$(LD) -o $@ $(LDFLAGS) $? $(LIBS)

calibration_april: calibration_april.o april_tag_board_generator.o
	$(LD) -o $@ $(LDFLAGS) $? $(LIBS)


.cpp.o:
	$(CXX) -c -o $@ $(CXXFLAGS) $<


clean:
	rm -f $(APPS)
	rm -f *.o
