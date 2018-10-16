#pragma once

#include <opencv2/core.hpp>

#include "AplCam/distortion/distortion_model.h"
#include "AplCam/sonar_pose.h"

namespace camera_calibration {

// Puts the relevant math in one place.
class SonarImageWarper {
public:

  // Standard API, give it a camera model and a sonar pose
  SonarImageWarper( Distortion::DistortionModel *, AplCam::SonarPose * );

  // Load camera model and pose from files
  SonarImageWarper( const std::string &cameraCalFile, const std::string &sonarPoseFile );

  cv::Vec2f sonarToImage(  float x, float y, float z );
  cv::Vec2f sonarToImage( const cv::Vec3f &s );

  Distortion::DistortionModel *cam( void ) { return _cam; }

protected:

  Distortion::DistortionModel *_cam;
  AplCam::SonarPose *_pose;

};

}
