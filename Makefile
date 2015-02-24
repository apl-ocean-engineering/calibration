
APPS := calibration calibration_artificial

all: $(APPS)


CXXFLAGS = -I/opt/opencv/include
LDFLAGS = $(CXXFLAGS) -L/opt/opencv/lib
LIBS = -lopencv_core -lopencv_calib3d -lopencv_features2d -lopencv_highgui -lopencv_imgproc
LD = g++


calibration: calibration.o
	$(LD) -o $@ $(LDFLAGS) $< $(LIBS)

calibration_artificial: calibration_artificial.o
	$(LD) -o $@ $(LDFLAGS) $< $(LIBS)

.cpp.o:
	$(CXX) -c -o $@ $(CXXFLAGS) $<


clean:
	rm -f $(APPS)
	rm -f *.o
