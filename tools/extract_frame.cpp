
#include <iostream>

#include <opencv2/highgui/highgui.hpp>

using namespace std;
using namespace cv;

int main( int argc, char **argv ) {

  if( argc < 2 ) {
    cout << "Usage: extract_frame <filename> <image> " << endl;
    exit(0);
  }

  string videofile( argv[argc-2] );
  string outfile( argv[argc-1] );

  VideoCapture capture;
capture.open( videofile.c_str() );

if( !capture.isOpened() ) {
  cout << "Could not initialize video capture for " << videofile << endl;
  exit(-1);
}

Mat img;
capture.read( img );

imwrite( outfile.c_str(), img );

}
