
#include <stdlib.h>
#include <stdio.h>

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

using AprilTags::TagDetection;

struct AlignmentOptions
{

  typedef enum { PLAYER, DETECTOR, NONE = -1 } Verb;

  // n.b. the default window should be a non-round number so you don't get an ambiguous number of second transitions...
  AlignmentOptions( int argc, char **argv )
    : window( 4.2 ), maxDelta( 5.0 ), lookahead(1.2),
    minSharedTags( 10 ),
    seekTo(0), offset(0),  waitKey(0), 
    standoffFrames( 100 ),
    offsetGiven(false),
    doExtract( false ),
    extractPath( "/tmp/extracted" ),
    verb( NONE )
  {
    string msg;
    if( parseArgv( argc, argv, msg ) == false ) {
      cout << msg << endl;
      exit(-1);
    }
  }


  float window, maxDelta, lookahead;
  int minSharedTags, seekTo, offset, waitKey, standoffFrames;
  bool offsetGiven, doExtract;
  string extractPath;

  Verb verb;

  string video1, video2;

  bool parseArgv( int argc, char **argv, string &msg )
  {
    static struct option long_options[] = {
      { "do-extract", optional_argument, NULL, 'e' },
      { "window", required_argument, NULL, 'w' },
      { "seek-to", required_argument, NULL, 's' },
      { "shared-tags", required_argument, NULL, 'f' },
      { "standoff-frames", required_argument, NULL, 'y' },
      { "wait-key", required_argument, NULL, 'k' },
      { "max-delta", required_argument, NULL, 'd'},
      { "offset", required_argument, NULL, 'o'},
      { "lookahead", required_argument, NULL, 'l'},
      { "help", no_argument, NULL, '?' },
      { 0, 0, 0, }
    };

    int indexPtr;
    int optVal;
    while( (optVal = getopt_long( argc, argv, "e::w:y:s:f:k:d:o:l:?", long_options, &indexPtr )) != -1 ) {
      switch(optVal) {
        case 'd':
          maxDelta = atof(optarg);
          break;
        case 'e':   // Extract images
          doExtract = true;
          if( optarg ) extractPath = optarg;
          break;
        case 'w':
          window = atof( optarg );
          break;
        case 'y':
          standoffFrames = atoi( optarg );
          break;
        case 's':
          seekTo = atoi( optarg );
          break;
        case 'f':
          minSharedTags = atoi( optarg );
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
    } else if( verbStr.compare( "extract" ) == 0 ) {
      verb = DETECTOR;
      doExtract = true;
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


class AlignStreamsMain {
  public:
    AlignStreamsMain( int argc, char **argv )
      : opts( argc, argv ),
      video0( opts.video1, opts.lookahead ), 
      video1( opts.video2, opts.lookahead ),
      sync( video0, video1 ),
      extractPath( opts.extractPath )
  {;}

    int go( void )
    {
      string error;

      if( opts.offsetGiven ) {
        cout << "Using user-supplied offset of " << opts.offset << " frames" << endl;
        sync.setOffset( opts.offset );
      } else {
        cout << "Estimating offset between videos" << endl;
        sync.bootstrap( opts.window, opts.maxDelta, opts.seekTo );
      }

      sync.rewind();

      if( opts.seekTo > 0 ) sync.seek( 0, opts.seekTo );

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

        //const int tagCount = 80;


        // Calculate the number of tags in common
        vector< pair< AprilTags::TagDetection, AprilTags::TagDetection > > pairs = findCommonPairs( tags[0], tags[1] );

        cout << "Found " << tags[0].size() << " and " << tags[1].size();
        cout << "  with " << pairs.size() << " tags in common" << endl;

        bool extracted = false;
        if( opts.doExtract ) {
          //          if( ((float)tags[0].size() / tagCount) > opts.minFractionOfSharedTags &&
          //              ((float)tags[1].size() / tagCount) > opts.minFractionOfSharedTags ) {

          if ( pairs.size() >= opts.minSharedTags  ) {
            cout << "!!! I'm doing something" << endl;
            extracted = true;

            imwrite( extractPath.video0( video0.frame(), video1.frame() ).c_str(), frame[0] );
            imwrite( extractPath.video1( video0.frame(), video1.frame() ).c_str(), frame[1] );

            Mat composite;
            sync.compose( frame[0], frame[1], composite );
            imwrite( extractPath.composite( video0.frame(), video1.frame() ).c_str(), composite );

            Mat discard;
            for( int i = 0; i < opts.standoffFrames && sync.nextCompositeFrame( discard ); ++i ) {;}
          }
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

      vector< pair< TagDetection, TagDetection > > findCommonPairs(
          vector< TagDetection > &a, vector< TagDetection > &b )
      {
        vector< pair< TagDetection, TagDetection > > pairs;

        // Brute force for now.  Assume no duplicates
        for( int i = 0; i < a.size(); ++i ) {
          for( int j = 0; j < b.size(); ++j ) {
            if( a[i].id == b[j].id ) {
              pairs.push_back( make_pair( a[i], b[j] ) );
            }
          }
        }
        return pairs;
      }


      private:
      AlignmentOptions opts;
      VideoLookahead video0, video1;
      KFSynchronizer sync;


      struct ExtractPath {
        ExtractPath( const string &root )
          : _root( root )
        {;}

        string _root;

        const string video0( int frame0, int frame1 )
        { return videoPath( 0, frame0, frame1 ); }
        const string video1( int frame0, int frame1 )
        { return videoPath( 1, frame0, frame1 ); }

        const string videoPath( int which, int frame0, int frame1 )
        {
          char path[40];
          snprintf( path, 39, "/video%d/frame%d_%06d_%06d.jpg", which, which, frame0, frame1 );

          string total( _root );
          total += path;
          mkdir_p(total);

          return total;

        }

        const string composite( int frame0, int frame1 )
        {
          char path[40];
          snprintf( path, 39, "/composite/composite_%06d_%06d.jpg", frame0, frame1 );

          string total( _root );
          total += path;
          mkdir_p(total);

          return total;
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



      ExtractPath extractPath;
    };

    int main( int argc, char **argv )
    {

      AlignStreamsMain main( argc, argv );

      exit( main.go() );
    }


