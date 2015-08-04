// Based on the dark channel code from code of:  https://github.com/Chrisawa/image-video-dehazing-OpenCV

#include "dark_channel.h"

#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/ximgproc.hpp>

#include <glog/logging.h>

using namespace cv;


Mat DarkChannelPrior::calculateRGBMin(const Mat &src)
{
  Mat rgbmin = Mat::zeros(src.size(), CV_8UC1);

  Point p;
  for(p.y=0 ; p.y<src.rows ; p.y++)
  {
    for(p.x=0 ; p.x<src.cols ; p.x++)
    {
      const Vec3b &intensity( src.at<Vec3b>(p) );
      rgbmin.at<uchar>(p) = min(min(intensity.val[0], intensity.val[1]), intensity.val[2]);
    }
  }

  return rgbmin;
}

Mat DarkChannelPrior::calculateDarkChannel( const Mat &src )
{
  Mat rgbMin( calculateRGBMin( src ) );

LOG(INFO) << "DarkChannelPrior::calculateDarkChannel";

  Mat blurred;
  const int patchSize = 5;
  erode(rgbMin, blurred, getStructuringElement( MORPH_RECT, Size( patchSize, patchSize )));
  return blurred;
}

//estimate airlight by the brightest dark channel pixel
int DarkChannelPrior::estimateA(  const Mat &DC)
{
  double minDC, maxDC;
  minMaxLoc(DC, &minDC, &maxDC);
  LOG(INFO) << "estimated airlight is: " << maxDC;
  return maxDC;
}



//estimate transmission map
Mat DarkChannelPrior::estimateTransmission(const Mat &DCP, int ac)
{
  double w = 0.75;
  Mat transmission = Mat::zeros(DCP.size(), CV_8UC1);
  Scalar intensity;

  for (int m=0; m<DCP.rows; m++)
  {
    for (int n=0; n<DCP.cols; n++)
    {
      intensity = DCP.at<uchar>(m,n);
      transmission.at<uchar>(m,n) = (1 - w * intensity.val[0] / ac) * 255;
    }
  }
  return transmission;
}


//dehazing foggy image
Mat DarkChannelPrior::calculateDehazed(const Mat &source, const Mat &t, int al)
{
  float tmin = 0.1;
  float tmax;

  //Scalar inttran;
  //Vec3b intsrc;
  Mat Priord = Mat::zeros(source.size(), CV_8UC3);

  for(int i=0; i<source.rows; i++)
  {
    for(int j=0; j<source.cols; j++)
    {
      float inttran = t.at<uchar>(i,j);
      const Vec3b &intsrc( source.at<Vec3b>(i,j) );
      tmax = std::max( inttran/255, tmin );

      for(int k=0; k<3; k++)
      {
        int val = abs((intsrc.val[k] - al) / tmax + al);
        Priord.at<Vec3b>(i,j)[k] = std::min( val, 255 );
      }
    }
  }
  return Priord;
}




void DarkChannelPrior::dehaze( const Mat &img, Mat &out )
{
  Mat transmission, darkChannel;
  int airlight;

  darkChannel = calculateDarkChannel(img);

  airlight = estimateA(darkChannel);
  transmission = estimateTransmission(darkChannel, airlight);
  out = calculateDehazed(img, transmission, airlight);

  imshow( "Prior dark channel", darkChannel );
  imshow( "Prior transmission", transmission );
  imshow( "Prior out", out );
  waitKey(0);

}


//===================================================================

Mat MedianDarkChannelPrior::calculateDarkChannel( const Mat &src )
{
  Mat rgbMin( calculateRGBMin( src ) );

  const int filterPatch = 5;

LOG(INFO) << "Median filter dark channel";

  Mat blurred;
  medianBlur(rgbMin, blurred, filterPatch);
  return blurred;
}

//===================================================================

Mat BGDarkChannelPrior::calculateRGBMin(const Mat &src)
{
  Mat rgbmin = Mat::zeros(src.size(), CV_8UC1);

  Point p;
  for(p.y=0 ; p.y<src.rows ; p.y++)
  {
    for(p.x=0 ; p.x<src.cols ; p.x++)
    {
      const Vec3b &intensity( src.at<Vec3b>(p) );

      // Assumes BGR ordering
      rgbmin.at<uchar>(p) = min(intensity.val[0], intensity.val[1]);
    }
  }

  return rgbmin;
}

//===================================================================

Vec3b ColorDarkChannelPrior::estimateAirlightColor( const Mat &img, const Mat &DC, InputArray bgMask )
{
  double minDC, maxDC;

  minMaxLoc(DC, &minDC, &maxDC, NULL, NULL, bgMask );

imshow("bgMask", bgMask );
imshow("original DC", DC );

  double thr = 0.9*(maxDC - minDC) + minDC;
  Mat dcMask;
  compare( DC, Scalar( thr ), dcMask, CMP_GE );

  Mat andMask;
  if( bgMask.empty() )
    andMask = dcMask;
  else
    bitwise_and( dcMask, bgMask, andMask );

  imshow( "estimated most distant", andMask );

  // mean
  Scalar m( mean( img, andMask ));

  return Vec3b( max(m[0],1.0),
                max(m[1],1.0),
                max(m[2],1.0) );
}


//estimate transmission map
Mat ColorDarkChannelPrior::estimateTransmission(const Mat &img, const Vec3b &al )
{
  const double w = 0.95;
  Mat transmission = Mat::zeros(img.size(), CV_8UC1);
  Vec3b intensity;
  Point p;

  for (p.y=0; p.y<img.rows; p.y++)
    for (p.x=0; p.x<img.cols; p.x++)
    {
      Vec3b intensity( img.at<Vec3b>(p) );
      float m[3];
      for( int i = 0; i < 3; ++i ) m[i] = intensity[i] * 1.0 / al[i];

      float minM = min( min( m[0], m[1] ), m[2] );

      transmission.at<uchar>(p) = (1 - w * minM) * 255;
    }

  return transmission;
}


//dehazing foggy image
Mat ColorDarkChannelPrior::calculateDehazed(const Mat &source, const Mat &t, const Vec3b &al )
{
  float tmin = 0.1;
  float tmax;
  Point p;

  //Scalar inttran;
  //Vec3b intsrc;
  Mat Priord = Mat::zeros(source.size(), CV_8UC3);

  for(p.y=0; p.y<source.rows; p.y++)
  {
    for(p.x=0; p.x<source.cols; p.x++)
    {
      float inttran = t.at<uchar>(p);
      const Vec3b &intsrc( source.at<Vec3b>(p) );
      tmax = std::max( inttran/255, tmin );

      for(int k=0; k<3; k++)
      {
        int val = abs((intsrc.val[k] - al[k]) / tmax + al[k]);
        Priord.at<Vec3b>(p)[k] = std::min( val, 255 );
      }
    }
  }
  return Priord;
}

void ColorDarkChannelPrior::dehaze( const Mat &img, Mat &out, InputArray bgMask )
{
  Mat transmission, darkChannel;
  Vec3b airlight;

  darkChannel = calculateDarkChannel(img);
  airlight = estimateAirlightColor(img, darkChannel, bgMask );

  LOG(INFO) << "Calculated airlight as " << airlight;

  transmission = estimateTransmission(img, airlight);
  out = calculateDehazed(img, transmission, airlight);

  imshow( "Dark channel", darkChannel );
  imshow( "Transmission", transmission );
  imshow( "Dehazed output", out );
  waitKey(0);

}

//===================================================================


//dehazing foggy image
Mat GuidedFilterDarkChannelPrior::calculateDehazed(const Mat &source, const Mat &t, int al)
{
  //Scalar inttran;
  //Vec3b intsrc;
  Mat Priord = Mat::zeros(source.size(), CV_8UC3);
  const int radius = 5;
  const double eps = 0.1;

  ximgproc::guidedFilter( source, t, Priord, radius, eps );

  return Priord;
}
