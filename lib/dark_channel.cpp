// Based on the dark channel code from code of:  https://github.com/Chrisawa/image-video-dehazing-OpenCV

#include "dark_channel.h"

#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/ximgproc.hpp>

#include <glog/logging.h>

using namespace cv;

//median filtered dark channel
Mat DarkChannelDehaze::getMedianDarkChannel(const Mat &src, int patch)
{
  Mat rgbmin = Mat::zeros(src.size(), CV_8UC1);
  Mat MDCP;

  for(int m=0; m<src.rows; m++)
  {
    for(int n=0; n<src.cols; n++)
    {
      const Vec3b &intensity( src.at<Vec3b>(m,n) );
      rgbmin.at<uchar>(m,n) = min(min(intensity.val[0], intensity.val[1]), intensity.val[2]);
    }
  }

  medianBlur(rgbmin, MDCP, patch);
  return MDCP;
}

//estimate airlight by the brightest pixel in dark channel (proposed by He et al.)
int DarkChannelDehaze::estimateA( const Mat &DC)
{
  double minDC, maxDC;
  minMaxLoc(DC, &minDC, &maxDC);
  LOG(INFO) << "estimated airlight is: " << maxDC;
  return maxDC;
}


//estimate transmission map
Mat DarkChannelDehaze::estimateTransmission(const Mat &DCP, int ac)
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
Mat DarkChannelDehaze::getDehazed(const Mat &source, const Mat &t, int al)
{
  float tmin = 0.1;
  float tmax;

  //Scalar inttran;
  //Vec3b intsrc;
  Mat dehazed = Mat::zeros(source.size(), CV_8UC3);

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
        dehazed.at<Vec3b>(i,j)[k] = std::min( val, 255 );
      }
    }
  }
  return dehazed;
}


DarkChannelDehaze::DarkChannelDehaze( const Mat &img, Mat &out )
{
  dehaze( img, out );
}

void DarkChannelDehaze::dehaze( const Mat &img, Mat &out )
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

  _darkChannel = getMedianDarkChannel(img, 5);
  _airlight = estimateA(_darkChannel);
  _transmission = estimateTransmission(_darkChannel, _airlight);
  out = getDehazed(img, _transmission, _airlight);

  imshow( "Dehaze dark channel", _darkChannel );
  imshow( "Dehaze transmission", _transmission );
  imshow( "Dehaze out", out );
  waitKey(0);

}


//===================================================================

GuidedFilterDarkChannelDehaze::GuidedFilterDarkChannelDehaze( const Mat &img, Mat &out )
: DarkChannelDehaze( img, out )
{ ; }


//dehazing foggy image
Mat GuidedFilterDarkChannelDehaze::getDehazed(const Mat &source, const Mat &t, int al)
{
  //Scalar inttran;
  //Vec3b intsrc;
  Mat dehazed = Mat::zeros(source.size(), CV_8UC3);
  const int radius = 5;
  const double eps = 0.1;

  ximgproc::guidedFilter( source, t, dehazed, radius, eps );

  return dehazed;
}
