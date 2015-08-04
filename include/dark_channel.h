#ifndef __DARK_CHANNEL_H__
#define __DARK_CHANNEL_H__

#include <opencv2/core.hpp>

using cv::Mat;
using cv::Vec3b;

class DarkChannelPrior {
public:

  DarkChannelPrior( void ) {;}

  DarkChannelPrior( const Mat &img, Mat &out )
  { dehaze( img, out );}

  virtual void dehaze( const Mat &img, Mat &out );

protected:

  virtual Mat calculateRGBMin(const Mat &src);
  virtual Mat calculateDarkChannel( const Mat &src );
  int estimateA( const Mat &DC);
  Mat estimateTransmission(const Mat &DCP, int ac);
  virtual Mat calculateDehazed(const Mat &source, const Mat &t, int al);

};

class MedianDarkChannelPrior : public DarkChannelPrior {
public:

  MedianDarkChannelPrior( void ) {;}

  MedianDarkChannelPrior( const Mat &img, Mat &out )
  { dehaze( img, out );}

protected:

  virtual Mat calculateDarkChannel(const Mat &src );

};

class BGDarkChannelPrior : public MedianDarkChannelPrior {
public:

  BGDarkChannelPrior( const Mat &img, Mat &out )
  { dehaze( img, out );}

protected:

  virtual Mat calculateRGBMin(const Mat &src );

};

class ColorDarkChannelPrior : public MedianDarkChannelPrior {
public:

  ColorDarkChannelPrior( const Mat &img, Mat &out, cv::InputArray bgMask = cv::noArray() )
  { dehaze( img, out, bgMask );}

  virtual void dehaze( const Mat &img, Mat &out, cv::InputArray bgMask = cv::noArray() );

protected:

  Vec3b estimateAirlightColor( const Mat &img, const Mat &DC, cv::InputArray bgMask = cv::noArray() );
  Mat estimateTransmission(const Mat &DCP, const Vec3b &ac);
  virtual Mat calculateDehazed(const Mat &source, const Mat &t, const Vec3b &ac);

};

class GuidedFilterDarkChannelPrior : public DarkChannelPrior {
public:

  GuidedFilterDarkChannelPrior( const Mat &img, Mat &out )
  { dehaze( img, out );}

protected:

  virtual Mat calculateDehazed(const Mat &source, const Mat &t, int al);

};

#endif
