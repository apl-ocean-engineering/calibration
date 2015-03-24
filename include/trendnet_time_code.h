
#include <opencv2/core/core.hpp>

#ifndef __TRENDNET_TIME_CODE_H__
#define __TRENDNET_TIME_CODE_H__

const cv::Rect timeCodeROI_1920x1080( 19, 8, 228, 14 );

cv::Mat timeCodeMask_1920x1080();
cv::Mat timeCodeUnmask_1920x1080();



#endif
