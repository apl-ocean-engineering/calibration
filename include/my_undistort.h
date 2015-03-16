
#ifndef __MY_UNDISTORT_H__
#define __MY_UNDISTORT_H__

#include <opencv2/core/core.hpp>

using cv::InputArray;
using cv::OutputArray;

void myUndistortPoints( InputArray _src, OutputArray _dst,
                          InputArray _cameraMatrix,
                          InputArray _distCoeffs,
                          InputArray _Rmat,
                          InputArray _Pmat );

#endif

