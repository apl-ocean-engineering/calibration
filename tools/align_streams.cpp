
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
#include "composite_canvas.h"

using namespace cv;
using namespace std;

using AprilTags::TagDetection;

using namespace AplCam;

struct AlignmentOptions
{

  typedef enum { PLAYER, EXTRACT, COMPOSITE, RECTIFY, VERB_NONE = -1 } Verb;
  typedef enum { TRIGGER_NONE = -1, DT, SHARED_TAGS } ExtractTrigger;

  // n.b. the default window should be a non-round number so you don't get an ambiguous number of second transitions...
  AlignmentOptions( int argc, char **argv )
    : window( 4.2 ), maxDelta( 3.0 ), lookahead(1.2),
    minSharedTags( 10 ),
    seekTo(0), offset(0),  waitKey(0), 
    extractDt( 30 ), 
    offsetGiven(false),
    outputPath( "/tmp/extracted" ),
    extractTrigger( DT ),
    verb( VERB_NONE )
  {
    string msg;
    if( parseArgv( argc, argv, msg ) == false ) {
      cout << msg << endl;
      exit(-1);
    }
  }


  float window, maxDelta, lookahead;
  int minSharedTags, seekTo, offset, waitKey, standoffFrames, extractDt;
  bool offsetGiven;
  string outputPath;
  ExtractTrigger extractTrigger;

  Verb verb;

  string video1, video2;

  void help( string &msg )
  {
    stringstream strm;
    strm << "  Usage: align_streams [options] <verb> <video1> <video2>" << endl;
    strm << "Attempts to automatically align two video streams, and optionally re-exports the result." << endl;
    strm << endl;
    strm << "Options: " << endl;

    int fw = 30;
    strm.setf( std::ios::left );
    strm << setw(fw) << "--seek-to [offset]" << "Start alignment processing and exporting at [offset] frames from start of videos" << endl;
    strm << setw(fw) << "--extract-trigger [type]" << "Set behavior for saving composite images: dt, tags" << endl;
    strm << setw(fw) << "--output-path [path]" << "Root of tree where outputs will be saved" << endl;

    msg = strm.str();
  }



  bool parseArgv( int argc, char **argv, string &msg )
  {
    static struct option long_options[] = {
      { "window", required_argument, NULL, 'w' },
      { "seek-to", required_argument, NULL, 's' },
      { "extract-trigger", required_argument, NULL, 'x' },
      { "extract-interval", required_argument, NULL, 'i' },
      { "shared-tags", required_argument, NULL, 'f' },
      { "wait-key", required_argument, NULL, 'k' },
      { "max-delta", required_argument, NULL, 'd'},
      { "offset", required_argument, NULL, 'O'},
      { "output-path", required_argument, NULL, 'o' },
      { "lookahead", required_argument, NULL, 'l'},
      { "help", no_argument, NULL, '?' },
      { 0, 0, 0, }
    };

    int indexPtr;
    int optVal;
    string str;
    while( (optVal = getopt_long( argc, argv, "O:e:x:w:y:i:s:f:k:d:o:l:?", long_options, &indexPtr )) != -1 ) {
      switch(optVal) {
        case 'O':
          offset = atoi( optarg );
          offsetGiven = true;
          break;
        case 'd':
          maxDelta = atof(optarg);
          break;
        case 'w':
          window = atof( optarg );
          break;
        case 'y':
          standoffFrames = atoi( optarg );
          break;
        case 'x':
          extractTrigger = parseExtractTrigger( optarg );
          break;
        case 'i':
          extractDt = atoi( optarg );
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
          outputPath = optarg;
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
    } else if( verbStr.compare( "extract" ) == 0 ) {
      verb = EXTRACT;
    } else if( verbStr.compare( "composite" ) == 0 ) {
      verb = COMPOSITE;
    } else if( verbStr.compare( "rectify" ) == 0 ) {
      verb = RECTIFY;
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

    if( (extractTrigger == TRIGGER_NONE) && verb == EXTRACT ) {
      msg = "Extract trigger unknown";
      return false;
    }

    if( verb == RECTIFY && outputPath.empty() ) {
      msg = "Must specify output path for rectify";
      return false;
    }


    return true;
  }

  ExtractTrigger parseExtractTrigger( const char *optarg )
  {
    string str( optarg );
    if( str.compare( "tags" ) == 0 ) 
      return SHARED_TAGS;
    else if( str.compare( "dt" ) == 0 )
      return DT;

        return TRIGGER_NONE;
  }

};


class AlignStreamsMain {
  public:
    AlignStreamsMain( int argc, char **argv )
      : opts( argc, argv ),
      video0( opts.video1, opts.lookahead ), 
      video1( opts.video2, opts.lookahead ),
      sync( video0, video1 ),
      outputPath( opts.outputPath )
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
        case AlignmentOptions::EXTRACT:
          retval = doExtract();
          break;
        case AlignmentOptions::COMPOSITE:
          retval = doComposite();
          break;
        case AlignmentOptions::RECTIFY:
          retval = doRectify();
          break;
        case AlignmentOptions::VERB_NONE:
        default:
          cout << "No verb selected, oh well." << endl;
          retval = 0;
      }

      return retval;
    }

    int doPlayer( void )
    {
      CompositeCanvas canvas;
      while( sync.nextCompositeFrame( canvas ) ) {
        imshow( "Composite", canvas );

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


    int doComposite( void )
    {
      // Need to generate first composite to get frame size
      CompositeCanvas composite;
      sync.nextCompositeFrame( composite );

      string outfile = outputPath.compositeVideo();
      //VideoWriter writer( outfile, CV_FOURCC('X','2','6','4'), 
      //VideoWriter writer( outfile, CV_FOURCC('M','J','P','G'),
      VideoWriter writer( outfile, CV_FOURCC('X','V','I','D'),
          std::min( video0.fps(), video1.fps() ), composite.size(), true );

      if( !writer.isOpened() ) {
        cerr << "Couldn't open video writer for \"" << outfile << "\"" << endl;
        return -1;
      }

      int count = 0;
      do { 
        writer << composite;
        ++count;
      } while( sync.nextCompositeFrame( composite ) ); 

      return true;
    }



    int doRectify( void )
    {
      // Need to generate first composite to get frame size
      CompositeCanvas composite;
      sync.nextCompositeFrame( composite );

      //VideoWriter writer( outfile, CV_FOURCC('X','2','6','4'), 
      //VideoWriter writer( outfile, CV_FOURCC('M','J','P','G'),
      VideoWriter writer0( outputPath.videoZero(), CV_FOURCC('X','V','I','D'),
                          std::min( video0.fps(), video1.fps() ), composite[0].size(), true );

      VideoWriter writer1( outputPath.videoOne(), CV_FOURCC('X','V','I','D'),
                          std::min( video0.fps(), video1.fps() ), composite[1].size(), true );


      if( !writer0.isOpened() ) {
        cerr << "Couldn't open video writer for \"" << outputPath.videoZero() << "\"" << endl;
        return -1;
      }

      if( !writer1.isOpened() ) {
        cerr << "Couldn't open video writer for \"" << outputPath.videoOne() << "\"" << endl;
        return -1;
      }

      int count = 0;
      do { 
        writer0 << composite[0];
        writer1 << composite[1];

        ++count;
      } while( sync.nextCompositeFrame( composite ) ); 

      return true;
    }


    int doExtract()
    {
      CompositeCanvas composite;
      int count = 0;
      while( sync.nextCompositeFrame( composite ) ) {

        if( opts.extractTrigger == AlignmentOptions::DT ) {
          if( (count % opts.extractDt) == 0 )
            saveComposite( composite );

        } else if( opts.extractTrigger == AlignmentOptions::SHARED_TAGS ) {


          Mat bw[2];
          cvtColor( composite[0], bw[0], CV_BGR2GRAY );
          cvtColor( composite[1], bw[1], CV_BGR2GRAY );

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

            //          if( ((float)tags[0].size() / tagCount) > opts.minFractionOfSharedTags &&
            //              ((float)tags[1].size() / tagCount) > opts.minFractionOfSharedTags ) {

            if ( pairs.size() >= opts.minSharedTags  ) {
              cout << "!!! I'm doing something" << endl;
              extracted = true;

              //imwrite( outputPath.video0( video0.frame(), video1.frame() ).c_str(), frame[0] );
              //imwrite( outputPath.video1( video0.frame(), video1.frame() ).c_str(), frame[1] );

              saveComposite( composite );

              //Mat composite;
              //sync.compose( frame[0], frame[1], composite );
              //imwrite( extractPath.composite( video0.frame(), video1.frame() ).c_str(), composite );

              CompositeCanvas discard;
              for( int i = 0; i < opts.extractDt && sync.nextCompositeFrame( discard ); ++i ) {;}
            }


          for( int j = 0; j < 2; ++j ) {
            for( int i = 0; i < tags[j].size(); ++i ) {
              tags[j][i].draw( composite[j] );
            }
          }
          } else {
            cerr << "Hm, no extraction trigger set." << endl;
            return -1;
          }

          imshow( "Composite", composite.scaled( 0.5 ) );

          int ch;
          ch = waitKey( opts.waitKey );

          if( ch == 'q' )
            break;



          //Mat shrunk;
          //sync.compose( frame[0], frame[1], shrunk, 0.5 );
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

          ++count;
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


      protected:

      void saveComposite( CompositeCanvas &composite )
      {
        imwrite( outputPath.compositeImages( video0.frame(), video1.frame() ).c_str(), composite );
      }


      private:
      AlignmentOptions opts;
      VideoLookahead video0, video1;
      KFSynchronizer sync;


      struct OutputPath {
        OutputPath( const string &root )
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

        const string compositeImages( int frame0, int frame1 )
        {
          char path[40];
          snprintf( path, 39, "/composite/composite_%06d_%06d.jpg", frame0, frame1 );

          string total( _root );
          total += path;
          mkdir_p(total);

          return total;
        }

        const string compositeVideo( void )
        {
          return _root + "/composite.avi";
        }

        // Todo.  Hardcoded.
        const string videoZero( void )
        {
          return _root + "/zero.avi";
        }

        const string videoOne( void )
        {
          return _root + "/one.avi";
        }

      } outputPath;

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


    };

    int main( int argc, char **argv )
    {

      AlignStreamsMain main( argc, argv );

      exit( main.go() );
    }


