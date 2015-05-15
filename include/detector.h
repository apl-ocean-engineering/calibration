

#ifndef __DETECTOR_H__
#define __DETECTOR_H__

#include <opencv2/core/core.hpp>


namespace CameraCalibration {

  using cv::Mat;

  struct Frame {
    Frame( int _f, Mat _m )
      : frame(_f), img( _m )
    {;}

    int frame;
    Mat img;
  };

  typedef vector< Frame > FrameVec_t;
  typedef pair< int, int64 > TimingData_t;
  typedef vector< TimingData_t > TimingDataVec_t;

}




#endif
