

#include "background_segmenter.h"

#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>

#include <glog/logging.h>

using namespace cv;

void BackgroundSegmenter::buildMask( void )
{
  LOG(INFO) << _img.size() << " " << _bg.size();
  assert( _img.size() == _bg.size() );
  assert( !_img.empty() );
  assert( !_bg.empty() );

  _mask = Mat::zeros( _img.size(), CV_8UC1 );

  Mat imgGrey, bgGrey;
  cvtColor( _img, imgGrey, CV_BGR2GRAY );
   cvtColor( _bg,  bgGrey, CV_BGR2GRAY );

  blur( imgGrey, imgGrey, cv::Size(3,3) );
  blur( bgGrey, bgGrey, cv::Size(3,3) );

  Mat diff( imgGrey.size(), imgGrey.type() );
  absdiff( imgGrey, bgGrey, diff );

  Mat diffFl;
diff.convertTo( diffFl, CV_32FC1);

  Mat scaled;
  divide( diffFl, bgGrey, scaled, 1.0/255.0, CV_32FC1 );

  const int x = 934, y = 599;
  LOG(INFO) << "ImgGrey: " << (int)imgGrey.at< unsigned char  >( y,x );
  LOG(INFO) << "BgGrey:  " << (int)bgGrey.at< unsigned char >( y,x );
  LOG(INFO) << "diff:    " << (int)diff.at< unsigned char >( y,x );
  LOG(INFO) << "scaled:  " << (int)scaled.at< unsigned char >( y,x );

imwrite("/tmp/imgGrey.png", imgGrey );
imwrite("/tmp/bgGrey.png", bgGrey );
imwrite("/tmp/diff.png", diff);

  imshow( "background_segmenter: img", imgGrey );
  imshow( "background_segmenter: bg", bgGrey );
  imshow( "background_segmenter: diff", diff );
  imshow( "background_segmenter: scaled", scaled );
}
