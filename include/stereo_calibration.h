
#ifndef __STEREO_CALIBRATION_H__
#define __STEREO_CALIBRATION_H__

#include <string>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace AplCam {

  using cv::Mat;

  class StereoCalibration {
    public:
      StereoCalibration() {;}

      void save( cv::FileStorage &fs ) const ;

      bool load( cv::FileStorage &fs );
      bool load( const std::string &filename );

      Mat E, F, R, t;

      static const std::string fundamentalTag, 
                   essentialTag, 
                   rotationTag, 
                   translationTag;
  };

  class StereoRectification {
    public:
      StereoRectification() {;}

      void save( cv::FileStorage &fs ) const ;

      bool load( cv::FileStorage &fs );
      bool load( const std::string &filename );

      Mat R[2], P[2];

      static const std::string rect0Tag, rect1Tag, 
                               proj0Tag, proj1Tag;
  };

}


#endif
