
#include <stdlib.h>
#include <stdio.h>

#include <iostream>
#include <iomanip>
#include <string>
#include <sstream>
#include <fstream>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <getopt.h>

//#include <gsl/gsl_randist.h>
//#include <gsl/gsl_cdf.h>
//
//#define THREADED_APRILTAG_DETECTION
//#ifdef THREADED_APRILTAG_DETECTION
//#include <boost/thread.hpp>
//#endif
//
//#ifndef USE_APRILTAGS
//#error "Will only compile with Apriltags support."
//#endif
//
//#include "AprilTags/TagDetector.h"
//#include "AprilTags/TagFamily.h"
//#include "AprilTags/Tag36h11.h"
//
//#include "file_utils.h"
//#include "trendnet_time_code.h"

#include "video.h"
#include "synchronizer.h"
#include "composite_canvas.h"

using namespace cv;
using namespace std;

//using AprilTags::TagDetection;

using namespace AplCam;

struct Options
{

  typedef enum { PLAYER, VERB_NONE = -1 } Verb;

  // n.b. the default window should be a non-round number so you don't get an ambiguous number of second transitions...
  Options( int argc, char **argv )
    : seekTo(0), waitKey(0), scale(-1.0),
    verb( VERB_NONE )
  {
    string msg;
    if( parseArgv( argc, argv, msg ) == false ) {
      cout << msg << endl;
      exit(-1);
    }
  }

  int seekTo, waitKey;
  float scale;

  Verb verb;

  string video;

  void help( string &msg )
  {
    stringstream strm;
    strm << "  Usage: composite_video [options] <verb> <video>" << endl;
    strm << "Attempts to automatically align two video streams, and optionally re-exports the result." << endl;
    strm << endl;
    strm << "Options: " << endl;

    int fw = 30;
    strm.setf( std::ios::left );
    strm << setw(fw) << "--seek-to [offset]" << "Start alignment processing and exporting at [offset] frames from start of videos" << endl;

    msg = strm.str();
  }



  bool parseArgv( int argc, char **argv, string &msg )
  {
    static struct option long_options[] = {
      { "seek-to", required_argument, NULL, 's' },
      { "wait-key", required_argument, NULL, 'k' },
      { "scale", required_argument, NULL, 'S' },
      { "help", no_argument, NULL, '?' },
      { 0, 0, 0, }
    };

    int indexPtr;
    int optVal;
    while( (optVal = getopt_long( argc, argv, "s:k:?", long_options, &indexPtr )) != -1 ) {
      switch(optVal) {
        case 's':
          seekTo = atoi( optarg );
          break;
        case 'S':
          scale = atof( optarg );
        case 'k':
          waitKey = atoi( optarg );
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
      msg = "Must specify verb and video file on command line";
      return false;
    }

    string verbStr( argv[optind++]);

    if( verbStr.compare( 0, 4, "play" ) == 0 ) {
      verb = PLAYER;
    } else {
      msg = "Don't understand the verb \"" + verbStr + "\"";
      return false;
    }

    video = argv[optind++];

    return validate( msg );
  }

  bool validate( string &msg )
  {
    if( !file_exists( video ) ) {
      msg = "File \'" + video + "\' doesn't exist";
      return false;
    }

    return true;
  }

};


class CompositeVideoMain {
  public:
    CompositeVideoMain( int argc, char **argv )
      : opts( argc, argv ),
      video( opts.video )
  {;}

    int go( void )
    {
      string error;
      
      if( ! video.isOpened() ) {
        cout << "Could not open video " << opts.video << endl;
        return -1;
      }

      video.rewind();
      if( opts.seekTo > 0 ) video.seek( opts.seekTo );

      int retval;
      switch( opts.verb ) {
        case Options::PLAYER:
          retval = doPlayer( );
          break;
        case Options::VERB_NONE:
        default:
          cout << "No verb selected, oh well." << endl;
          retval = 0;
      }

      return retval;
    }

    int doPlayer( void )
    {
      CompositeCanvas canvas;
      while( video.read( canvas ) ) {
        if( opts.scale > 0 )
          imshow("Composite", canvas.scaled( opts.scale  ) );
        else
          imshow( "Composite", canvas );

        int ch;
        ch = waitKey( opts.waitKey );

        if( ch == 'q' )
          break;
        else if (ch == 'R')
          video.rewind();
      }

      return 0;
    }


  private:
    Options opts;
    CompositeVideo video;


};

int main( int argc, char **argv )
{

  CompositeVideoMain main( argc, argv );

  exit( main.go() );
}


