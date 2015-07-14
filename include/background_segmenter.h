#ifndef __BACKGROUND_SEGMENTER_H__
#define __BACKGROUND_SEGMENTER_H__

#include <string>
#include <opencv2/core.hpp>

using std::string;
using cv::Mat;
using cv::Point2i;

class BackgroundSegmenter {
public:

  BackgroundSegmenter( )
  {;}

  BackgroundSegmenter( const Mat &img, const Mat &bg )
  : _img( img ), _bg( bg )
  { buildMask(); }

  BackgroundSegmenter( const Mat &img, const string &bgFile )
  : _img( img ), _bg( cv::imread( bgFile ) )
  {buildMask();}

  void setImage( const Mat &mat )
  { _img = mat; }

  void setBackground( const Mat &mat )
  {
    _bg = mat;
    buildMask();
  }

  void loadBackground( const string &bgFile )
  {
    setBackground( cv::imread( bgFile ) );
  }


  bool isForeground( const Point2i &pt )
  {
return true;
    if( _bg.empty() ) return true;

    return maskAt( pt );
  }


protected:

  void buildMask( void )
  {
    LOG(INFO) << _img.size() << " " << _bg.size();
    assert( _img.size() == _bg.size() );
    assert( !_img.empty() );
    assert( !_bg.empty() );

    _mask = Mat::zeros( _img.size(), CV_8UC1 );

    Mat diff;  //( _img.size(), CV_32FC3 );
cv::subtract( _img, _bg, diff, cv::noArray(), CV_32FC3 );
    //diff = _img - _bg;

    imshow( "background_segmenter", diff );

  }

  uint8_t maskAt( const Point2i &pt )
  {
    return _mask.at<uint8_t>( pt.y, pt.x );
  }


  Mat _img, _bg, _mask;

};

#endif
