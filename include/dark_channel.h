#ifndef __DARK_CHANNEL_H__
#define __DARK_CHANNEL_H__

#include <opencv2/core.hpp>

using cv::Mat;

class DarkChannelDehaze {
public:

  DarkChannelDehaze( const Mat &img, Mat &out );

  void dehaze( const Mat &img, Mat &out );

protected:
  
  Mat _transmission, _darkChannel;
  int _airlight;

};


#endif
