#ifndef __IMAGE_DETECTIONS_H__
#define __IMAGE_DETECTIONS_H__

#include <string>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include "detection_base.h"

using cv::Mat;

class ImageDetection : public DetectionBase {
public:
  ImageDetection( const std::string &timestamp )
  : DetectionBase( timestamp )
  {;}

  virtual ~ImageDetection()
  {;}

virtual Mat &draw( Mat &img ) = 0;
};

class ImageSphereDetection : public ImageDetection {
public:
  ImageSphereDetection( const std::string &timestamp, float x, float y,  float radius )
  : ImageDetection( timestamp ),
    _x(x), _y(y), _radius(radius)
  {;}

virtual Mat &draw( Mat &img )
{
  cv::circle( img, cv::Point( _x, _y ), _radius, cv::Scalar(0,255,255), 1 );
  cv::circle( img, cv::Point( _x, _y ), 3, cv::Scalar( 0,255,255), -1 );

  return img;
}

protected:

  float _x, _y, _radius;

};

#endif
