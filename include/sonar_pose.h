#ifndef __SONAR_POSE_H__
#define __SONAR_POSE_H__

#include <string>

#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>

#include <Eigen/Core>

#include  <glog/logging.h>

#include "sonar_types.h"

using Eigen::Vector6d;

using cv::Vec3f;
using cv::Matx33f;

class SonarPose {
 public:
  SonarPose( const Vector6d &p );
  SonarPose( const Vec3f &rot, const Vec3f &trans );

  bool write( const std::string &filename );
  static SonarPose *Load( const std::string &filename );

  const Vec3f &rot( void ) const { return _rot; }
  Matx33f rotMat( void ) const 
  {
    Matx33f rotMatx;

    cv::Rodrigues( _rot, rotMatx );
    return rotMatx;
  }
  Vec3f euler( void ) const
  {
    cv::Matx33f qx, qy, qz, Rq, Qq;
    cv::RQDecomp3x3( rotMat(), Rq, Qq, qx, qy, qz );

    return Vec3f( acos( qx(1,1) ),
                  acos( qy(0,0) ),
                  acos( qz(0,0) ) );
  }



  const Vec3f &trans( void ) const { return _trans; }


 protected:
  Vec3f _rot, _trans;

};

#endif
