
#include <stdlib.h>

#include <iostream>
#include <string>
#include <sstream>
#include <fstream>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <getopt.h>

#include <gsl/gsl_randist.h>
#include <gsl/gsl_cdf.h>

#ifdef USE_FFTS
#include <ffts.h>
#endif

#include "AprilTags/TagDetector.h"
#include "AprilTags/TagFamily.h"
#include "AprilTags/Tag36h11.h"

#include "file_utils.h"
#include "trendnet_time_code.h"

#include "video.h"
#include "synchronizer.h"

using namespace cv;
using namespace std;

#define MAKE_NORMFILE
#ifdef MAKE_NORMFILE
ofstream normFile;
#endif


struct AlignmentOptions
{
  // n.b. the default window should be a non-round number so you don't get an ambiguous number of second transitions...
  AlignmentOptions( void )
    : window( 4.2 ), maxDelta( 5.0 ), lookahead(1.2), minFraction(0.5),
    offset(0),  waitKey(1), 
    offsetGiven(false)
  {;}


  float window, maxDelta, lookahead, minFraction;
  int offset, waitKey;
  bool offsetGiven;

  string video1, video2;

  bool parseArgv( int argc, char **argv, string &msg )
  {
    static struct option long_options[] = {
      { "window", true, NULL, 'w' },
      { "min-fraction", true, NULL, 'w' },
      { "wait-key", true, NULL, 'k' },
      { "max-delay", true, NULL, 'd'},
      { "offset", true, NULL, 'o'},
      { "lookahead", true, NULL, 'l'},
      { "help", false, NULL, '?' },
      { 0, 0, 0, }
    };

    int indexPtr;
    int optVal;
    while( (optVal = getopt_long( argc, argv, ":w:k:d:o:l:?", long_options, &indexPtr )) != -1 ) {
      switch(optVal) {
        case 'd':
          maxDelta = atof(optarg);
          break;
        case 'f':
          minFraction = atof( optarg );
          break;
        case 'w':
          window = atof( optarg );
          break;
        case 'k':
          waitKey = atoi( optarg );
          break;
        case 'o':
          offset = atoi( optarg );
          offsetGiven = true;
          break;
        case 'l':
          lookahead = atof( optarg );
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



int main( int argc, char **argv )
{
  string error;
  AlignmentOptions opts;
  if( opts.parseArgv( argc, argv, error ) != true ) {
    if( !error.empty() ) cout << error  << endl;
    exit(-1);
  }

  VideoLookahead video[2] = { VideoLookahead( opts.video1, opts.lookahead ), VideoLookahead( opts.video2, opts.lookahead ) };
  KFSynchronizer sync( video[0], video[1] );

  if( opts.offsetGiven )
    sync.setOffset( opts.offset );
  else
    sync.bootstrap( opts.window, opts.maxDelta );

  sync.rewind();
  sync.seek( 0, 500 );

  AprilTags::TagDetector detector( AprilTags::tagCodes36h11 );

  Mat frame[2];
  while( sync.nextSynchronizedFrames( frame[0], frame[1] ) ) {

    Mat bw[2];
    cvtColor( frame[0], bw[0], CV_BGR2GRAY );
    cvtColor( frame[1], bw[1], CV_BGR2GRAY );

    vector<AprilTags::TagDetection> tags[2];
    tags[0] = detector.extractTags( bw[0] );
    tags[1] = detector.extractTags( bw[1] );

    const int tagCount = 35;

    cout << "Found " << tags[0].size() << " and " << tags[1].size() << endl;


    if( ((float)tags[0].size() / tagCount) > opts.minFraction &&
        ((float)tags[1].size() / tagCount) > opts.minFraction ) {
      cout << "!!! I'm doing something" << endl;
    }

    for( int j = 0; j < 2; ++j ) {
      for( int i = 0; i < tags[j].size(); ++i ) {
        tags[j][i].draw( frame[j] );
      }
    }

    Mat shrunk;
    sync.compose( frame[0], frame[1], shrunk, 0.5 );
    imshow( "Composite", shrunk );

    int ch;
    ch = waitKey( opts.waitKey );

    if( ch == 'q' )
      break;
    else if (ch == ',')
      sync.scrub(-2);
    else if (ch == '[')
      sync.advanceToNextTransition( 0 );
    else if (ch == ']')
      sync.advanceToNextTransition( 1 );
    else if (ch == 'R')
      sync.rewind();
    else if (ch == 'l')
      sync.advanceOnly( 0 );
    else if (ch == 'r')
      sync.advanceOnly( 1 );

  }


  exit(0);
}


