
#include "trendnet_time_code.h"

using namespace cv;

//const Rect timeCodeROI_1920x1080( 20, 8, 227, 14 );

Mat timeCodeMask_1920x1080( void )
{
  Mat mask( Mat::zeros( 1080, 1920, CV_8UC1 ) );
  Mat roi( mask, timeCodeROI_1920x1080 );
  roi.setTo( 1 );
  return mask;
}

Mat timeCodeUnmask_1920x1080( void )
{
  Mat mask( Mat::ones( 1080, 1920, CV_8UC1 ) );
  Mat roi( mask, timeCodeROI_1920x1080 );
  roi.setTo( 0 );
  return mask;
}

