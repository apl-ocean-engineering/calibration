
#include <iostream>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "trendnet_time_code.h"

using namespace std;
using namespace cv;

int main( int argc, char **argv ) {

  if( argc < 2 ) {
    cout << "Usage: extract_frame <filename>" << endl;
    exit(0);
  }

  string videofile( argv[argc-1] );

  cout << "Opening " << videofile << endl;
  VideoCapture capture;
  capture.open( videofile.c_str() );

  if( !capture.isOpened() ) {
    cout << "Could not initialize video capture for " << videofile << endl;
    exit(-1);
  }

  Mat img, prev;
  Mat prevTimeCode;
  int frame = 0;
  while( capture.read( img ) ) {
    ++frame;
    Mat imgTC( img, timeCodeROI_1920x1080 ), imgTimeCode;
    cvtColor( imgTC, imgTimeCode, CV_BGR2GRAY );

    if( !prev.empty() )  {
      double norm = cv::norm( imgTimeCode, prevTimeCode, NORM_L2 );

      cout  << frame << " : " << norm << endl;

      // Sadly, a heuristic
      if( norm > 500 ) {
        Mat composite( Mat::zeros( imgTimeCode.size().height + 2 + prevTimeCode.size().height,
            std::max( imgTimeCode.size().width, prevTimeCode.size().width ), 
            imgTimeCode.type() ) );
        Mat beforeROI( composite, Rect( Point2i(0,0), prevTimeCode.size() ) );
        prevTimeCode.copyTo( beforeROI );

        Mat afterROI( composite, Rect( Point2i( 0, prevTimeCode.size().height+2 ), imgTimeCode.size() ) );
        imgTimeCode.copyTo( afterROI );

        char outFilename[40];
        sprintf( outFilename, "/tmp/watch/frame_%05d.png", frame );
        imwrite( outFilename, composite );
      }


    }


    prevTimeCode = imgTimeCode;
    prev = img;
  }
}
