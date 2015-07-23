#ifndef __SONAR_IMAGE_WARPER_H__
#define __SONAR_IMAGE_WARPER_H__

#include "opencv2/core.hpp"

#include "distortion/distortion_model.h"

#include "sonar_pose.h"

// Puts the relevant math in one place.
class SonarImageWarper {
public:

  // Standard API, give it a camera model and a sonar pose
  SonarImageWarper( Distortion::DistortionModel *, SonarPose * );

  // Load camera model and pose from files
  SonarImageWarper( const std::string &cameraCalFile, const std::string &sonarPoseFile );

  cv::Vec2f sonarToImage(  float x, float y, float z );
  cv::Vec2f sonarToImage( const cv::Vec3f &s );

  Distortion::DistortionModel *cam( void ) { return _cam; }

protected:

  Distortion::DistortionModel *_cam;
  SonarPose *_pose;

};

#endif
