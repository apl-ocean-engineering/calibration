
#ifndef __STEREO_CALIBRATION_H__
#define __STEREO_CALIBRATION_H__

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace AplCam {

  using cv::Mat;

  class StereoCalibration {
    public:
      StereoCalibration() {;}

      void save( cv::FileStorage &fs ) const ;

      Mat E, F, R, t;
  };

}


#endif
