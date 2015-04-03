
#ifndef __CALIBRATION_RESULT_H__
#define __CALIBRATION_RESULT_H__

#include <opencv2/core/core.hpp>

#include "types.h"

namespace AplCam {

  using cv::FileStorage;


  struct Result {
    Result()
      : rms(-1) 
    {;}

    virtual void serialize( FileStorage &fs )
    {
      fs << "rms" << rms;
    }

    double rms;
  };


  struct CalibrationResult : public Result {
    CalibrationResult()
      : Result(), success(false),
      totalTime(-1.0), residual(-1.0),
      numPoints(-1), numImages(-1),
      rvecs(),
      tvecs(),
      status()
    {;}

    void resize( size_t sz )
    {
      success = false;
      totalTime = -1.0;
      residual = rms = -1.0;
      rvecs.resize( sz, Vec3d(0,0,0) );
      tvecs.resize( sz, Vec3d(0,0,0) );
      status.resize( sz, false );
    }

    virtual void serialize( FileStorage &fs )
    {
      Result::serialize(fs);

      fs << "success" << success;
      fs << "totalTime" << totalTime;
      fs << "numPoints" << numPoints;
      fs << "numImage" << numImages;
      fs << "residual" << residual;
    }

    bool success;
    double totalTime, rms, residual;
    int numPoints, numImages;

    RotVec rvecs;
    TransVec tvecs;
    vector< bool > status;
  };

}

#endif
