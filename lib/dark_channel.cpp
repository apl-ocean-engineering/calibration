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

//estimate airlight by the brightest pixel in dark channel (proposed by He et al.)
int DarkChannelPrior::estimateA( const Mat &DC)
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
  //Mat fog = imread("tiananmen1.bmp");
  // Mat darkChannel;
  // Mat T;
  // Mat fogfree;
  // Mat beforeafter = Mat::zeros(fog.rows, 2 * fog.cols, CV_8UC3);
  // Rect roil (0, 0, fog.cols, fog.rows);
  // Rect roir (fog.cols, 0, fog.cols, fog.rows);
  // int Airlight;
  // namedWindow("before and after", CV_WINDOW_AUTOSIZE);

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
