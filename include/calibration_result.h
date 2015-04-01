
#ifndef __CALIBRATION_RESULT_H__
#define __CALIBRATION_RESULT_H__

#include "types.h"

namespace AplCam {

 struct CalibrationResult {
   CalibrationResult()
     : success(false),
       totalTime(-1.0), rms(-1.0), residual(-1.0),
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


   bool success;
   double totalTime, rms, residual;

   RotVec rvecs;
   TransVec tvecs;
   vector< bool > status;
 };

}

#endif
