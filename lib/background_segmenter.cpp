

#include "background_segmenter.h"

#include <opencv2/core/mat.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>

#include <glog/logging.h>

#include "trendnet_time_code.h"

using namespace cv;

void BackgroundSegmenter::buildMask( void )
{
  LOG(INFO) << _img.size() << " " << _bg.size();
  assert( _img.size() == _bg.size() );
  assert( !_img.empty() );
  assert( !_bg.empty() );

  _mask = Mat::zeros( _img.size(), CV_8UC1 );

const float spatialWindowRadius = 10,
      colorWindowRadius = 10,
derivThreshold = 0.2;

  Mat posterized;
  pyrMeanShiftFiltering( _img, posterized, spatialWindowRadius,
                          colorWindowRadius, 2 );

  Mat imgGrey, imgGreyF, blurredF, laplacianF;
  cvtColor( posterized, imgGrey, cv::COLOR_BGR2GRAY );
  imgGrey.convertTo( imgGreyF, CV_32F );

  // Unsharp operation
  cv::GaussianBlur( imgGreyF, blurredF, cv::Size(5,5), 0);
  cv::Laplacian(blurredF, laplacianF, CV_32F);

  float weight = 0.4, scale = 0.2;
  Mat_<float> sharpenedF = 1.5f * imgGreyF
                            - 0.5f * blurredF
                            - weight * imgGreyF.mul(scale * laplacianF);

   Mat sharpened;
   sharpenedF.convertTo( sharpened, CV_8U);

  Mat edges, normEdges, thresholded;
  cv::Sobel( sharpened, edges, CV_32F, 1, 1, 3 );
  edges = abs(edges);

  normalize( edges, normEdges, 1.0, 0.0, NORM_MINMAX );

threshold( normEdges, thresholded, derivThreshold, 1.0, THRESH_BINARY );

// Blank out any changes around the timecode...
Mat timeCodeROI( thresholded, TimeCode_1920x1080::timeCodeOvermask );
timeCodeROI.setTo( 0 );

  Mat morphed;
thresholded.copyTo( morphed );   // Extraneous, but preserves thresholded for debugging
Mat elem = getStructuringElement( MORPH_RECT, Size( 3,3 ) );
dilate(  morphed, morphed, elem, Point(-1,-1), 2 );
erode(  morphed, morphed, elem, Point(-1,-1), 3 );

elem = getStructuringElement( MORPH_RECT, Size( 11,11 ) );
dilate( morphed, morphed, elem, Point(-1,-1), 1 );
erode( morphed, morphed, elem, Point(-1,-1), 1 );

dilate( morphed, morphed, elem, Point(-1,-1), 3 );


morphed.convertTo( _mask, CV_8U, 255.0 );

  imshow( "build_mask: posterized", posterized );
  imshow( "build_mask: imgGrey", imgGrey );
  imshow( "build_mask: sobel edges", edges );
  imshow( "build_mask: normalized sobel edges", normEdges );
  imshow( "build_mask: thresholded sobel edges", thresholded );
  imshow( "build_mask:  morphed", morphed );
  imshow( "build_mask:  morphed", _mask );

}


#ifdef no
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
#endif
