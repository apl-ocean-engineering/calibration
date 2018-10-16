
#include <iostream>

#include <opencv2/highgui/highgui.hpp>

using namespace std;
using namespace cv;

#include "trendnet_time_code.h"


int main( int argc, char **argv )
{
  if( argc < 2 ) {
    cout << "Usage: trendnet_time_code_test <image>" << endl;
    exit(0);
  }

  string infile( argv[argc-1] );
  Mat image = imread( infile.c_str() );

  if( image.empty() ) {
    cout << "Couldn't read file " << infile << endl;
    exit(-1);
  }

  if( image.cols != 1920 || image.rows != 1080 ) {
    cout << "Image is incorrect size (" << image.cols << " x " << image.rows << ")" << endl;
    exit(-1);
  }

  // Extract code into an ROI
  Mat timecode( image, timeCodeROI_1920x1080 );
  imwrite( "timecode.png", timecode );

  // Use the mask
  Mat clone( image.clone() );
  clone.setTo( 0, timeCodeMask_1920x1080() );
  imwrite( "blacked_out_timecode.png", clone );

  clone = image.clone();
  clone.setTo( 0, timeCodeUnmask_1920x1080() );
  imwrite( "blacked_out_everything_else.png", clone );

}
