#ifndef __DETECTION_H__
#define __DETECTION_H__

#include <string>
#include <vector>

#include <opencv2/core/core.hpp>

#include "board.h"


struct SharedPoints
{
  std::vector< cv::Point2f > aPoints, bPoints;
  std::vector< cv::Point3f > worldPoints;
};

struct Detection 
{
  Detection(  )
    : found(false), points(), corners(), ids() {;}

  bool found;
  std::vector< cv::Point2f > points;
  std::vector< cv::Point3f > corners;
  std::vector< int > ids;

int size( void ) const { return points.size(); }

  virtual void calculateCorners( const Board &board );
  virtual void drawCorners(  const Board &board, cv::Mat &view ) const;

  virtual void writeCache( const Board &board, const std::string &cacheFile );

  static Detection *loadCache( const std::string &cacheFile );
  static SharedPoints sharedWith( Detection &a, Detection &b );
};

#ifdef USE_APRILTAGS
struct AprilTagsDetection : public Detection
{
  AprilTagsDetection( vector< AprilTags::TagDetection > det )
    : Detection(), _det(det) {;}

  vector< AprilTags::TagDetection > _det;

  virtual void calculateCorners( const AprilTagsBoard &board );


};
#endif


#endif
