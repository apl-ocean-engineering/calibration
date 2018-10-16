//
//   Don't work on this file, integrate it's functionality into stereo_processor!!
//
//

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
#include <opencv2/calib3d/calib3d.hpp>

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
#include "stereo_calibration.h"
using namespace AplCam;

#include "distortion/distortion_model.h"
#include "distortion/camera_factory.h"
using namespace Distortion;

#include "feature_tracker.h"

using namespace cv;
using namespace std;

//using AprilTags::TagDetection;

struct Options
{

  typedef enum { PLAYER, TRACKER, VERB_NONE = -1 } Verb;
  typedef enum { FAST, FEATURES_NONE = -1 } Features;

  // n.b. the default window should be a non-round number so you don't get an ambiguous number of second transitions...
  Options( int argc, char **argv )
    : seekTo(0), waitKey(0), scale(-1.0), doRectify( false ), doDenseStereo( false ), doUndistort( false ),
    suppressFeaturesInTimecode( true ),
    dataDir("../data"), stereoPair(),
    features( FEATURES_NONE ),
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
  bool doRectify, doDenseStereo, doUndistort, suppressFeaturesInTimecode;
  string dataDir, stereoPair;
  Features features;

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
      { "data-directory", required_argument, NULL, 'D' },
      { "stereo-pair", required_argument, NULL, 'P' },
      { "rectify", no_argument, NULL, 'R'},
      { "undistort", no_argument, NULL, 'U' },
      { "disparity", no_argument, NULL, 'B' },
      { "features", required_argument, NULL, 'F'},
      { "help", no_argument, NULL, '?' },
      { 0, 0, 0, }
    };

    int indexPtr;
    int optVal;
    stringstream sstr;
    while( (optVal = getopt_long( argc, argv, "BD:F:P:Rs:k:?", long_options, &indexPtr )) != -1 ) {
      switch(optVal) {
        case 'B':
          doDenseStereo = true;
          doRectify = true;
          break;
        case 'D':
          dataDir = optarg;
          break;
        case 'F':
          features = parseFeatures( optarg );
          if( features == FEATURES_NONE ) {
            sstr << "Had trouble parsing feature type \"" << optarg << "\"";
            msg = sstr.str();
            return false;
          }
          break;
        case 'P':
          stereoPair = optarg;
          break;
        case 'R':
          doRectify = true;
          break;
        case 's':
          seekTo = atoi( optarg );
          break;
        case 'S':
          scale = atof( optarg );
          break;
        case 'U':
          doUndistort = true;
          break;
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
    } else if( verbStr.compare( 0, 5, "track" ) == 0 ) {
      verb = TRACKER;
    } else {
      msg = "Don't understand the verb \"" + verbStr + "\"";
      return false;
    }

    video = argv[optind++];

    return validate( msg );
  }

  Features parseFeatures( const string &optarg )
  {
    if( optarg.compare("fast") == 0 )
      return FAST;


    return FEATURES_NONE;
  }

  bool validate( string &msg )
  {
    if( !file_exists( video ) ) {
      msg = "File \'" + video + "\' doesn't exist";
      return false;
    }

    if( stereoPair.size() > 0 && !file_exists( stereoCalibrationFile() ) ) {
      msg = "Stereo calibation file " + stereoCalibrationFile() + " doesn't exist";
      return false;
    }

    return true;
  }

  string stereoCalibrationFile( void )
  {
    return dataDir + "/stereo_pairs/" + stereoPair + ".yml";
  }

};


struct TxKeyPointInTimecode {
  TxKeyPointInTimecode( float scale = 1.0 )
    : _scale( 1.0 / scale ) { ;}
  float _scale;

  bool operator()( const KeyPoint &kp )
  {
    return TimeCode_1920x1080::timeCodeROI.contains( kp.pt * _scale );
  }
};

struct TxKeyPointScaler {
  TxKeyPointScaler( float scale = 1.0 )
    : _scale( 1.0 / scale ) { ;}
  float _scale;

  KeyPoint operator()( const KeyPoint &kp )
  {
    return KeyPoint( kp.pt * _scale, kp.size * _scale, kp.angle, kp.response, kp.octave );
  }
};




class DoFeatureTracker {
  public:
    DoFeatureTracker( const Options &options )
      : opts( options ), _tracker()
    {;}

    bool processCompositeImage( CompositeCanvas &canvas );

  protected:
    const Options &opts;

    FeatureTracker _tracker[2];
};

bool DoFeatureTracker::processCompositeImage( CompositeCanvas &canvas )
{
  vector<KeyPoint> keypoints[2];

  float scale = 0.25;
  Mat pyrTwoMat( Size( canvas.size().width * scale, canvas.size().height * scale ), CV_8UC1 );
  CompositeCanvas pyrTwo( pyrTwoMat );

  for( int k = 0; k < 2; ++k ) {
    Mat grey;
    cvtColor( canvas[k], grey, CV_BGR2GRAY );
    resize( grey, pyrTwo[k], pyrTwo[k].size(), 0, 0, INTER_LINEAR );

    FAST( pyrTwo[k], keypoints[k], 10, true );

    if( opts.suppressFeaturesInTimecode ) {
      std::remove_if( keypoints[k].begin(), keypoints[k].end(), TxKeyPointInTimecode( scale ) );
    }


    _tracker[k].update( pyrTwo[k], keypoints[k], canvas[k], scale );

    //_tracker[k].drawTracks( canvas[k], scale );

    //vector<KeyPoint> scaledKeypoints;
    //std::transform( keypoints[k].begin(), keypoints[k].end(), back_inserter(scaledKeypoints), TxKeyPointScaler( scale ) );
    //drawKeypoints( canvas[k], scaledKeypoints, canvas[k], Scalar(0,0,255),
    //    cv::DrawMatchesFlags::DRAW_OVER_OUTIMG | cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
  }


  imshow("TrackerOutput", canvas );

  int ch;
  ch = waitKey( opts.waitKey );

  if( ch == 'q' )
    return false;

  return true;
}

class CompositeVideoMain {
  public:
    CompositeVideoMain( int argc, char **argv )
      : opts( argc, argv ), video( opts.video ), _tracker( opts )
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


      if( loadCalibrationFiles() == false ) return -1;


      int retval;
      switch( opts.verb ) {
        case Options::PLAYER:
        case Options::TRACKER:
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
      Mat map[2][2];

      CompositeCanvas canvas;
      while( video.read( canvas ) ) {

        if( opts.verb == Options::TRACKER ) {
          if( _tracker.processCompositeImage( canvas ) == false ) break;
        } else {
          if( opts.doRectify )  {
            for( int k = 0; k < 2; ++k ) {
              // JIT construct the map because we need the imageSize, which isn't
              // available until the first frame has been loaded.
              if( map[k][0].empty() || map[k][1].empty() )
                cameras[k]->initUndistortRectifyMap( sRect.R[k], sRect.P[k],
                    canvas[k].size(), CV_32FC1, map[k][0], map[k][1] );

              remap( canvas[k], canvas[k], map[k][0], map[k][1], INTER_LINEAR );
            }

            if( opts.doDenseStereo ) {
              doDenseStereo( canvas );
            }
          } else if( opts.doUndistort ) {
            doUndistort( canvas );
          }

          if( opts.features != Options::FEATURES_NONE ) {
            doFeatures( canvas );
          }

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
      }

      return 0;
    }


  private:
    Options opts;
    CompositeVideo video;

    StereoCalibration sCal;
    StereoRectification sRect;
    DistortionModel *cameras[2];

    DoFeatureTracker _tracker;

    bool doDenseStereo( const CompositeCanvas &canvas )
    {
      int numberOfDisparities = ((canvas[0].size().width / 8) + 15) & -16;
      int SADWindowSize = 0;

 //     StereoBM bm;
 //     StereoSGBM sgbm;


 //     // bm.state->roi1 = roi1;
 //     // bm.state->roi2 = roi2;
 //     // bm.state->preFilterCap = 31;
 //     // bm.state->SADWindowSize = SADWindowSize > 0 ? SADWindowSize : 9;
 //     bm.state->minDisparity = 0;
 //     bm.state->numberOfDisparities = numberOfDisparities;
 //     bm.state->textureThreshold = 10;
 //     bm.state->uniquenessRatio = 15;
 //     bm.state->speckleWindowSize = 100;
 //     bm.state->speckleRange = 32;
 //     bm.state->disp12MaxDiff = 1;

 //     sgbm.preFilterCap = 63;
 //     sgbm.SADWindowSize = 3;

 //     //int cn = canvas.canvas.channels();

 //     sgbm.P1 = 8*sgbm.SADWindowSize*sgbm.SADWindowSize;
 //     sgbm.P2 = 32*sgbm.SADWindowSize*sgbm.SADWindowSize;
 //     sgbm.minDisparity = 0;
 //     sgbm.numberOfDisparities = numberOfDisparities;
 //     sgbm.uniquenessRatio = 10;
 //     //sgbm.speckleWindowSize = bm.state->speckleWindowSize;
 //     //sgbm.speckleRange = bm.state->speckleRange;
 //     sgbm.disp12MaxDiff = 1;
 //     sgbm.fullDP = false;

 //     Mat scaled[2];

 //     if( opts.scale > 0 ) {
 //       resize( canvas[0], scaled[0], Size(), opts.scale, opts.scale, cv::INTER_LINEAR );
 //       resize( canvas[1], scaled[1], Size(), opts.scale, opts.scale, cv::INTER_LINEAR );
 //       cvtColor( scaled[0], scaled[0], CV_BGR2GRAY );
 //       cvtColor( scaled[1], scaled[1], CV_BGR2GRAY );
 //     } else {
 //       cvtColor( canvas[0], scaled[0], CV_BGR2GRAY );
 //       cvtColor( canvas[1], scaled[1], CV_BGR2GRAY );
 //     }


 //     Mat disparity;
 //     int64 t = getTickCount();
 //     //bm( scaled[0], scaled[1], disparity );
 //     sgbm( scaled[0], scaled[1], disparity );
 //     t = getTickCount() - t;
 //     cout << "Dense stereo elapsed time (ms): " <<  t * 1000 / getTickFrequency() << endl;

 //     Mat disp8;
 //     disparity.convertTo(disp8, CV_8U, 255/(numberOfDisparities*16.));

 //     imshow( "disparity", disp8 );

      return true;
    }

    bool doUndistort( CompositeCanvas &canvas )
    {
      for( int k = 0; k < 2; ++k )
        cameras[k]->undistortImage( canvas[k], canvas[k] );

      return true;
    }

    bool doFeatures( CompositeCanvas &canvas )
    {
      vector<KeyPoint> keypoints[2];

      if( opts.features == Options::FAST ) {
        for( int k = 0; k < 2; ++k ) {
          Mat grey;
          cvtColor( canvas[k], grey, CV_BGR2GRAY );
          FAST( grey, keypoints[k], 10, true );
        }
      }

      for( int k = 0; k < 2; ++k )  {
        if( opts.suppressFeaturesInTimecode ) {
          std::remove_if( keypoints[k].begin(), keypoints[k].end(), TxKeyPointInTimecode() );
        }

        drawKeypoints( canvas[k], keypoints[k], canvas[k], Scalar(0,0,255),
            cv::DrawMatchesFlags::DRAW_OVER_OUTIMG | cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
      }

      return true;
    }



    bool loadCalibrationFiles( void )
    {
      // If stereo pair is set, assume we'll need it
      if( opts.stereoPair.size() > 0 ) {
        FileStorage fs( opts.stereoCalibrationFile(), FileStorage::READ );
        if( !fs.isOpened() ) {
          cerr << "Trouble opening stereo calibration file " + opts.stereoCalibrationFile() << endl;
          return false;
        }
        if( !sCal.load( fs ) ) {
          cerr << "Trouble loading stereo calibration from " + opts.stereoCalibrationFile() << endl;
          return false;
        }
        if( !sRect.load( fs ) ) {
          cerr << "Trouble loading stereo rectification from " + opts.stereoCalibrationFile() << endl;
          return false;
        }

        string calFile[2];
        fs["calibration_0"] >> calFile[0];
        fs["calibration_1"] >> calFile[1];

        for( int k =0; k < 2; ++k ) {
          // I should be smart and manipulate the path of the calibration files to looks
          // in the current data directory, but I can't be bothered right now
          if( !file_exists( calFile[k] ) ) {
            cerr << "Can't find calibration file " + calFile[k] << endl;
            return false;
          }

          cameras[k] = CameraFactory::LoadDistortionModel( calFile[k] );

          if( cameras[k] == NULL) {
            cerr << "Error loading calibration file " + calFile[k] << endl;
            return false;
          }
        }

      }

      return true;
    }

};

int main( int argc, char **argv )
{

  CompositeVideoMain main( argc, argv );

  exit( main.go() );
}
