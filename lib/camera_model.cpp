
#include "distortion_model.h"

namespace Distortion {

  using namespace std;
  using namespace cv;

  double Camera::calibrate( const ObjectPointsVecVec &objectPoints, 
      const ImagePointsVecVec &imagePoints, const Size& image_size,
      vector< Vec3d > &rvecs, 
      vector< Vec3d > &tvecs,
      int flags,
      cv::TermCriteria criteria )
  {
    CalibrationResult result;
    calibrate( objectPoints, imagePoints, image_size, result, flags, criteria );

    rvecs.clear();
    std::copy( result.rvecs.begin(), result.rvecs.end(), back_inserter( rvecs ) );

    tvecs.clear();
    std::copy( result.tvecs.begin(), result.tvecs.end(), back_inserter( tvecs ) );

    return result.rms;
  }

  // This does the "prep work", then doCalibrate is the virtual "dirty work" for each distortion model
  bool Camera::calibrate( const ObjectPointsVecVec &objectPoints, 
      const ImagePointsVecVec &imagePoints, const Size& image_size,
      CalibrationResult &result,
      int flags,
      cv::TermCriteria criteria )
  {
    result.resize( objectPoints.size() );

    const int minPoints = 3;

    for( int i = 0; i < objectPoints.size(); ++i ) {
      if( objectPoints[i].size() > minPoints && objectPoints[i].size() == imagePoints[i].size() ) {
        result.status[i] = true;
      }
    }

    doCalibrate( objectPoints, imagePoints, image_size, result, flags, criteria );

    return result.success;
  }





};

