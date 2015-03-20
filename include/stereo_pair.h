
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

  using cv::Mat;
  using cv::Rect;

  double stereoCalibrate( ObjectPointsVecVec _objectPoints,
      ImagePointsVecVec _imagePoints1,
      ImagePointsVecVec _imagePoints2,
      PinholeCamera &cam1, PinholeCamera &cam2,
      Size imageSize, OutputArray _Rmat, OutputArray _Tmat,
      OutputArray _Emat, OutputArray _Fmat, 
      cv::TermCriteria criteria,
      int flags );

  void stereoRectify( const PinholeCamera &cam1, const PinholeCamera &cam2,
                          const Size &imageSize, const Mat &_Rmat, const Mat &_Tmat,
                          Mat &_Rmat1, Mat &_Rmat2,
                          Mat &_Pmat1, Mat &_Pmat2,
                          Mat &_Qmat, int flags,
                          double alpha, const Size &newImageSize,
                          Rect &validPixROI1, Rect &validPixROI2 );
  
   //void stereoRectify( const PinholeCamera &cam1, const PinholeCamera &cam2,
   //                       const Size &imageSize, const Mat &_Rmat, const Mat &_Tmat,
   //                       Mat &_Rmat1, Mat &_Rmat2,
   //                       Mat &_Pmat1, Mat &_Pmat2,
   //                       Mat &_Qmat, int flags,
   //                       double alpha, const Size &newImageSize )
   //{ Rect roi1, roi2;
   //  stereoRectify( cam1, cam2, imageSize, _Rmat, _Tmat, _Rmat1, _Rmat2, _Pmat1, _Pmat2, _Qmat, flags, alpha, newImageSize, roi1, roi2 ); }


}

#endif
