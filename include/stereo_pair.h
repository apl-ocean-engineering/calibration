
#ifndef __STEREO_PAIR_H__
#define __STEREO_PAIR_H__

#include <opencv2/core/core.hpp>

#include "distortion_model.h"

namespace Distortion {

  using Distortion::ObjectPointsVecVec;
  using Distortion::ImagePointsVecVec;
  using Distortion::PinholeCamera;

  using cv::Size;
  using cv::OutputArray;

  double stereoCalibrate( ObjectPointsVecVec _objectPoints,
      ImagePointsVecVec _imagePoints1,
      ImagePointsVecVec _imagePoints2,
      const PinholeCamera &cam1, const PinholeCamera &cam2,
      Size imageSize, OutputArray _Rmat, OutputArray _Tmat,
      OutputArray _Emat, OutputArray _Fmat, 
      cv::TermCriteria criteria,
      int flags );


}

#endif
