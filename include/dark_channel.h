#ifndef __DARK_CHANNEL_H__
#define __DARK_CHANNEL_H__

#include <opencv2/core.hpp>

using cv::Mat;

class DarkChannelPrior {
public:

DarkChannelPrior( void ) {;}

  DarkChannelPrior( const Mat &img, Mat &out )
  { dehaze( img, out );}


  void dehaze( const Mat &img, Mat &out );

protected:

  virtual Mat calculateRGBMin(const Mat &src);
virtual Mat calculateDarkChannel( const Mat &src );
  int estimateA( const Mat &DC);
  Mat estimateTransmission(const Mat &DCP, int ac);
  virtual Mat calculateDehazed(const Mat &source, const Mat &t, int al);

};

class MedianDarkChannelPrior : public DarkChannelPrior {
public:

  MedianDarkChannelPrior( const Mat &img, Mat &out )
  { dehaze( img, out );}

protected:

  virtual Mat calculateDarkChannel(const Mat &src );

};

class BGDarkChannelPrior : public DarkChannelPrior {
public:

  BGDarkChannelPrior( const Mat &img, Mat &out )
  { dehaze( img, out );}

protected:

  virtual Mat calculateRGBMin(const Mat &src );

};

class GuidedFilterDarkChannelPrior : public DarkChannelPrior {
public:

  GuidedFilterDarkChannelPrior( const Mat &img, Mat &out )
  { dehaze( img, out );}

protected:

  virtual Mat calculateDehazed(const Mat &source, const Mat &t, int al);

};

#endif
