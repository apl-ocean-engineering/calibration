#ifndef __SONAR_IMAGE_WARPER_H__
#define __SONAR_IMAGE_WARPER_H__

#include "opencv2/core.hpp"

#include "distortion_model.h"
#include "sonar_pose.h"

class SonarImageWarper {
public:

  SonarImageWarper( Distortion::DistortionModel *, SonarPose * );
  SonarImageWarper( const std::string &cameraCalFile, const std::string &sonarPoseFile );

  cv::Vec2f sonarToImage(  float x, float y, float z );
  cv::Vec2f sonarToImage( const cv::Vec3f &s );

protected:

  Distortion::DistortionModel *_cam;
  SonarPose *_pose;

};

#endif
