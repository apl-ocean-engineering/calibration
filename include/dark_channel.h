#ifndef __DARK_CHANNEL_H__
#define __DARK_CHANNEL_H__

#include <opencv2/core.hpp>

using cv::Mat;

class DarkChannelDehaze {
public:

  DarkChannelDehaze( const Mat &img, Mat &out );

  void dehaze( const Mat &img, Mat &out );

protected:

Mat getMedianDarkChannel(const Mat &src, int patch);
int estimateA( const Mat &DC);
Mat estimateTransmission(const Mat &DCP, int ac);
virtual Mat getDehazed(const Mat &source, const Mat &t, int al);

  Mat _transmission, _darkChannel;
  int _airlight;

};

class GuidedFilterDarkChannelDehaze : public DarkChannelDehaze {
public:

  GuidedFilterDarkChannelDehaze( const Mat &img, Mat &out );



protected:

virtual Mat getDehazed(const Mat &source, const Mat &t, int al);

};

#endif
