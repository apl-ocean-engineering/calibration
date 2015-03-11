
#include <stdlib.h>
#include <getopt.h>
#include <time.h>

#include <iostream>
#include <iomanip>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "trendnet_time_code.h"

using namespace std;
using namespace cv;

enum Mode { PRETRIGGER, ARMED, WAIT };

int main( int argc, char **argv ) {

  int opt;
  string path("/tmp/watch"), prefix("save");

  while( (opt = getopt( argc, argv, "p:d:")) != -1  ) {
    switch( opt ) {
      case 'p':
        prefix = optarg;
        break;
      case 'd':
        path = optarg;
        break;
      case '?':
        cout << "Don't recognize option \"" << optopt << "\'"<< endl;
        exit(-1);
    }
  }


  if( optind >= argc ) {
    cout << "Usage: extract_frame <filename>" << endl;
    exit(0);
  }




  // Currently static, should be read from file (?)
  const float consecutiveSec = 0.25, standoffSec = 1;
  enum Mode state = PRETRIGGER; 

  string videofile( argv[optind] );

  cout << "Opening " << videofile << endl;
  VideoCapture capture;
  if( isnumber(videofile.c_str()[0]) )
    capture.open( atoi( videofile.c_str() ) );
  else
    capture.open( videofile.c_str() );


  double fps = capture.get( CV_CAP_PROP_FPS );
  if( fps == 0 ) fps = 30;
  const int consecutive = ceil( consecutiveSec * fps ),
        standoff = ceil( standoffSec * fps );

  cout << "Waits for " << consecutive << " consecutive frames" << endl;

  if( !capture.isOpened() ) {
    cout << "Could not initialize video capture for " << videofile << endl;
    exit(-1);
  }

  Mat img, grey, prev, scaled;
  Mat prevTimeCode;
  int frame = 0, imgCount = 0, counter = 0;
  bool flash = false;

  while( capture.read( img ) ) {
    ++frame;
    cvtColor( img, grey, CV_BGR2GRAY );

    Mat imgTC( grey, timeCodeROI_1920x1080 ), imgTimeCode;
    imgTC.setTo( 0 );

    Mat shrunk;
    resize( grey, shrunk, Size(), 0.25, 0.25, INTER_AREA );
    flip( shrunk, scaled, 1 );

    if( !prev.empty() ) {

      double norm = cv::norm( scaled, prev, NORM_L2 ); // * scaled.size().area();

      cout << frame << " : " << state << "," << counter << " : " << 
        std::setw(10) << std::setprecision(1) << std::fixed << norm << endl;

      // Implement a kind of hysteresis..
      int motionLevel = (state == ARMED ? 1500 : 1500);
      if( norm > motionLevel ) {
        // Motion

        switch( state ) {
          case PRETRIGGER:
            state = ARMED;
            counter = 0;
            break;
          case ARMED:
            counter = 0;
            break;
          case WAIT:
            if( ++counter > standoff ) {
              state = ARMED;
              counter = 0;
            }
            break;
        }

      } else {
        // No motion

        switch( state ) {
          case PRETRIGGER: 
            break;
          case ARMED:
            if( ++counter > consecutive ) {
              char timecode[40], filename[256];
              time_t tt;
              time( &tt );
              strftime( timecode, 39, "%y%m%d_%H%M%S", localtime( &tt ) );
              snprintf( filename, 255, "%s/%s_%s.png", path.c_str(), prefix.c_str(), timecode );
              cout << "Save to " << filename << endl;
              imwrite( filename, img );
              flash = true;

              state = WAIT;
              counter = 0;
            }
            break;
          case WAIT:
            break;
        }


      }


    }

    imshow("diff", scaled-prev );

    // Must be a deep copy or prev never changes
    scaled.copyTo( prev );

    // Put this after assignment above .. lets me invert in place when I "flash"
    if( flash ) bitwise_not( scaled, scaled );
    imshow("Video", scaled );
    waitKey( flash ? 1000 : 1 );
    flash = false;


  }

}
