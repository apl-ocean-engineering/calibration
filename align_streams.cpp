
#include <iostream>
#include <string>
#include <sstream>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <getopt.h>

#include "file_utils.h"
#include "trendnet_time_code.h"

using namespace cv;
using namespace std;

struct AlignmentOptions
{
  AlignmentOptions( void )
    : window( 5.0 ), maxDelta( 5.0 )
  {;}


  float window, maxDelta;
  string video1, video2;

  bool parseArgv( int argc, char **argv, string &msg )
  {
    static struct option long_options[] = {
      { "window", true, NULL, 'w' },
      { "max-delay", true, NULL, 'd'},
      { "help", false, NULL, '?' },
      { 0, 0, 0, }
    };

    int indexPtr;
    int optVal;
    while( (optVal = getopt_long( argc, argv, ":w:d:?", long_options, &indexPtr )) != -1 ) {
      switch(optVal) {
        case 'd':
          maxDelta = atof(optarg);
          break;
        case 'w':
          window = atof( optarg );
          break;
        case '?':
          help( msg );
          return false;
          break;
        default:
          stringstream strm;
          strm << "Unknown option \'" << optopt << "\'";
          msg = strm.str();
          return false;
          break;
      }

    }

    if( (argc - optind) < 2 ) {
      msg = "Must specify two video files on command line";
      return false;
    }

    video1 = argv[optind++];
    video2 = argv[optind];

    return validate( msg );
  }

  bool validate( string &msg )
  {
    if( !file_exists( video1 ) ) {
      msg = "File \'" + video1 + "\' doesn't exist";
      return false;
    }
    if( !file_exists( video2 ) ) {
      msg = "File \'" + video2 + "\' doesn't exist";
      return false;
    }

    return true;
  }

  void help( string &msg )
  {
    stringstream strm;
    strm << "Help!" << endl;

    msg = strm.str();
  }
};



class Video
{
  public:
    Video( const string &file )
      : capture( file.c_str() ),filename( file )
    {;}

    string filename;
    VideoCapture capture;

    string dump( void ) 
    {
      stringstream strm;

      strm << "File " << filename << ": " << \
        capture.get( CV_CAP_PROP_FRAME_WIDTH ) << " x " << capture.get( CV_CAP_PROP_FRAME_HEIGHT ) << ", ";
      strm << capture.get( CV_CAP_PROP_FPS ) << " fps.";

      return strm.str();
    }
};


int main( int argc, char **argv )
{
  string error;
  AlignmentOptions opts;
  if( opts.parseArgv( argc, argv, error ) != true ) {
    if( !error.empty() ) cout << error  << endl;
    exit(-1);
  }

  Video video[2] = { Video( opts.video1 ), Video( opts.video2 ) };

  for( int i = 0; i < 2; ++i ) {
    if( !video[i].capture.isOpened() ) {
      cerr << "Can't open video " << i << endl;
      exit(-1);
    }

    cout << video[i].dump() << endl;
  }




  exit(0);
}

