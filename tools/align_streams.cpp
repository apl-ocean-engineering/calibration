
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

#define THREADED_APRILTAG_DETECTION
#ifdef THREADED_APRILTAG_DETECTION
#include <boost/thread.hpp>
#endif

#ifndef USE_APRILTAGS
#error "Will only compile with Apriltags support."
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

struct AlignmentOptions
{

  typedef enum { PLAYER, DETECTOR, NONE = -1 } Verb;

  // n.b. the default window should be a non-round number so you don't get an ambiguous number of second transitions...
  AlignmentOptions( int argc, char **argv )
    : window( 4.2 ), maxDelta( 5.0 ), lookahead(1.2),
    minFractionOfSharedTags( 0.5 ),
    seekTo(0), offset(0),  waitKey(0), offsetGiven(false),
    verb( NONE )
  {
    string msg;
    if( parseArgv( argc, argv, msg ) == false ) {
      cout << msg << endl;
      exit(-1);
    }
  }


  float window, maxDelta, lookahead, minFractionOfSharedTags;
  int seekTo, offset, waitKey;
  bool offsetGiven;

  Verb verb;

  string video1, video2;

  bool parseArgv( int argc, char **argv, string &msg )
  {
    static struct option long_options[] = {
      { "window", true, NULL, 'w' },
      { "shared-fraction", true, NULL, 'f' },
      { "seek-to", true, NULL, 's' },
      { "wait-key", true, NULL, 'k' },
      { "max-delay", true, NULL, 'd'},
      { "offset", true, NULL, 'o'},
      { "lookahead", true, NULL, 'l'},
      { "help", false, NULL, '?' },
      { 0, 0, 0, }
    };

    int indexPtr;
    int optVal;
    while( (optVal = getopt_long( argc, argv, ":w:s:f:k:d:o:l:?", long_options, &indexPtr )) != -1 ) {
      switch(optVal) {
        case 'd':
          maxDelta = atof(optarg);
          break;
        case 'w':
          window = atof( optarg );
          break;
        case 's':
          seekTo = atoi( optarg );
          break;
        case 'f':
          minFractionOfSharedTags = atof( optarg );
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

    if( (argc - optind) < 3 ) {
      msg = "Must specify verb and two video files on command line";
      return false;
    }

    string verbStr( argv[optind++]);

    if( verbStr.compare( 0, 4, "play" ) == 0 ) {
      verb = PLAYER;
    } else if( verbStr.compare( "detect" ) == 0 ) {
      verb = DETECTOR;
    } else {
      msg = "Don't understand the verb \"" + verbStr + "\"";
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


#ifdef THREADED_APRILTAG_DETECTION
struct AprilTagDetectorCallable
{
  AprilTagDetectorCallable( vector<AprilTags::TagDetection> &detections, Mat &image )
    : _detections( detections ),
    _img( image ),
    _detector( AprilTags::tagCodes36h11 )
  {;}

  vector<AprilTags::TagDetection> &_detections;
  Mat &_img;
  AprilTags::TagDetector _detector;

  void operator()( void )
  {
    _detections = _detector.extractTags( _img );
  }

};
#endif


class AlignStreamsMain {
  public:
    AlignStreamsMain( int argc, char **argv )
      : opts( argc, argv ),
      video0( opts.video1, opts.lookahead ), 
      video1( opts.video2, opts.lookahead ),
      sync( video0, video1 )
  {
  }

    int go( void )
    {
      string error;

      if( opts.offsetGiven ) {
        cout << "Using user-supplied offset of " << opts.offset << " frames" << endl;
        sync.setOffset( opts.offset );
      } else {
        cout << "Estimating offset between videos" << endl;
        sync.bootstrap( opts.window, opts.maxDelta );
      }

      sync.rewind();

      if( opts.seekTo != 0 ) sync.seek( 0, opts.seekTo );

      int retval;
      switch( opts.verb ) {
        case AlignmentOptions::PLAYER:
          retval = doPlayer( );
          break;
        case AlignmentOptions::DETECTOR:
          retval = doDetector( );
          break;
        case AlignmentOptions::NONE:
        default:
          cout << "No verb selected, oh well." << endl;
          retval = 0;
      }

      return retval;
    }

    int doPlayer( void )
    {
      Mat img;
      while( sync.nextCompositeFrame( img ) ) {
        imshow( "Composite", img );

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

      return 0;
    }

    int doDetector( )
    {
      Mat frame[2];
      while( sync.nextSynchronizedFrames( frame[0], frame[1] ) ) {

        Mat bw[2];
        cvtColor( frame[0], bw[0], CV_BGR2GRAY );
        cvtColor( frame[1], bw[1], CV_BGR2GRAY );

        vector<AprilTags::TagDetection> tags[2];

#ifdef THREADED_APRILTAG_DETECTION
        boost::thread detector0( AprilTagDetectorCallable( tags[0], bw[0] ) ),
          detector1( AprilTagDetectorCallable( tags[1], bw[1] ) );

        detector0.join();
        detector1.join();

#else
        AprilTags::TagDetector detector( AprilTags::tagCodes36h11 );
        tags[0] = detector.extractTags( bw[0] );
        tags[1] = detector.extractTags( bw[1] );
#endif

        const int tagCount = 35;

        cout << "Found " << tags[0].size() << " and " << tags[1].size() << endl;


        if( ((float)tags[0].size() / tagCount) > opts.minFractionOfSharedTags &&
            ((float)tags[1].size() / tagCount) > opts.minFractionOfSharedTags ) {
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
        //else if (ch == ',')
        //  sync.scrub(-2);
        //else if (ch == '[')
        //  sync.advanceToNextTransition( 0 );
        //else if (ch == ']')
        //  sync.advanceToNextTransition( 1 );
        //else if (ch == 'R')
        //  sync.rewind();
        //else if (ch == 'l')
        //  sync.advanceOnly( 0 );
        //else if (ch == 'r')
        //  sync.advanceOnly( 1 );

      }

      return 0;
    }


  private:
    AlignmentOptions opts;
    VideoLookahead video0, video1;
    KFSynchronizer sync;

};

int main( int argc, char **argv )
{

  AlignStreamsMain main( argc, argv );

  exit( main.go() );
}


