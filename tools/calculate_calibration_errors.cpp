
#include <stdlib.h>
#include <getopt.h>
#include <time.h>

#include <iostream>
#include <iomanip>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <kchashdb.h>

#include "board.h"
#include "detection.h"
#include "detection_db.h"
#include "file_utils.h"
#include "camera_factory.h"
#include "trendnet_time_code.h"
using namespace AplCam;

using namespace std;
using namespace cv;
using kyotocabinet::HashDB;

using namespace Distortion;

struct CalcOpts {
  public:
    CalcOpts()
      : seekTo(0), intervalFrames(-1), waitKey( 1 ), 
      intervalSeconds( -1 ),
      dataDir("data"),
      boardName(), 
      doDisplay( false )
  {;}


    int seekTo, intervalFrames, waitKey;
    float intervalSeconds;
    string dataDir;
    string boardName;
    bool doDisplay;

    string refVideo;
    string calibFile;

    const string boardPath( void )
    { return dataDir + "/boards/" + boardName + ".yml"; }

    const string tmpPath( const string &file )
    { return dataDir + "/tmp/" + file; }

    const string cachePath( const string &file = "" ) const
    { return dataDir + "/cache/" + file; }

    //== Option parsing and help ==

    string help( void )
    {
      stringstream strm;
      const int w = 20;

      strm <<  "This is a tool for extracting images from a video file." << endl;
      strm << setw( w ) <<  "--data-directory, -d" << "Set location of data directory." << endl;
      strm << setw( w ) << "--do-display, -x" << "Do display video" << endl;
      //    "Usage: calibration\n"
      //    "     -d <data directory>      # Specify top-level directory for board/camera/cache files.\n"
      //    "     --board,-b <board_name>    # Name of calibration pattern\n"
      //    "     --camera, -c <camera_name> # Name of camera\n"
      //    "     --seek-to <frame>          # Seek to specified frame before starting\n"
      //    //     "     [-d <delay>]             # a minimum delay in ms between subsequent attempts to capture a next view\n"
      //    //     "                              # (used only for video capturing)\n"
      //    //     "     [-o <out_camera_params>] # the output filename for intrinsic [and extrinsic] parameters\n"
      //    //     "     [-op]                    # write detected feature points\n"
      //    //     "     [-oe]                    # write extrinsic parameters\n"
      //    //     "     [-zt]                    # assume zero tangential distortion\n"
      //    //     "     [-a <aspectRatio>]      # fix aspect ratio (fx/fy)\n"
      //    //     "     [-p]                     # fix the principal point at the center\n"
      //    //     "     [-v]                     # flip the captured images around the horizontal axis\n"
      //    //     "     [-V]                     # use a video file, and not an image list, uses\n"
      //    //     "                              # [input_data] string for the video file name\n"
      //    //     "     [-su]                    # show undistorted images after calibration\n"
      //    "     [input_data]             # list of files to use\n"
      //    "\n" );
      //printf("\n%s",usage);
      //printf( "\n%s", liveCaptureHelp );

      return strm.str();
    }

    string unknownOption( int opt )
    {
      stringstream strm;
      strm << "Unknown option \"" << opt << "\"";
      return strm.str();
    }


    bool parseOpts( int argc, char **argv, string &msg )
    {
      static struct option long_options[] = {
        { "data-directory", true, NULL, 'd' },
        { "board", true, NULL, 'b' },
        { "seek-to", true, NULL, 's' },
        { "interval-frames", true, NULL, 'i' },
        { "interval-seconds", true, NULL, 'I' },
        { "do-display", no_argument,  NULL, 'x' },
        { "help", false, NULL, '?' },
        { 0, 0, 0, 0 }
      };


      if( argc < 2 )
      {
        msg =  help();
        return false;
      }

      int indexPtr;
      int optVal;
      while( (optVal = getopt_long( argc, argv, "b:c:d:i:s:x?", long_options, &indexPtr )) != -1 ) {
        switch( optVal ) {
          case 'd':
            dataDir = optarg;
            break;
          case 'b':
            boardName = optarg;
            break;
          case 'i':
            intervalFrames = atoi( optarg );
            break;
          case 'I':
            intervalSeconds = atof( optarg );
            break;
          case 's':
            seekTo = atoi( optarg );
            break;
          case 'x':
            doDisplay = true;
            break;
          case '?': 
            msg = help();
            return false;
            break;
          default:
            msg = unknownOption( optopt );
            return false;
        }
      }

      if( optind == argc )
      {
        msg = help();
        return false;
      }

      if( optind > argc-2 ) {
        msg = "Need to specify reference video file and calibration file on command line";
        return false;
      }

      refVideo = argv[optind++];
      calibFile = argv[optind];

      if( intervalFrames > 0 && intervalSeconds > 0 ) {
        msg = "Can't specify both interval frames and interval seconds at the same time.";
        return false;
      }

      return validate( msg );
    }

    bool validate( string &msg )
    {
      return true;
    }

};




class CalculateReprojectionMain 
{
  public:
    CalculateReprojectionMain( CalcOpts &options )
      : opts( options )
    {;}


    int run( void ) {
      if( opts.doDisplay ) namedWindow( "BuildDb" );

      //board = Board::load( opts.boardPath(), opts.boardName );
      //if( !board ) {
      //  cerr << "Couldn't open board from " << opts.boardPath() << endl;
      //  return -1;
      //}

      DistortionModel *dist = CameraFactory::LoadDistortionModel( opts.calibFile );
      if( !dist ) {
        cerr << "Couldn't load distortion model " << opts.calibFile << endl;
        return -1;
      }

      if( ! db.open( opts.cachePath(), opts.refVideo ) ) {
        cerr << "Open error: " << db.error().name() << endl;
        return -1;
      }

      // Two modes of operation.  If intervalFrame == 0, uses all keys in db
      // otherwise, starts at seekTo and iterated by intervalFrame until it 
      // reaches the end

      string videoSource( opts.refVideo );
      VideoCapture vid( videoSource );
      if( !vid.isOpened() ) {
        cerr << "Couldn't open video source \"" << videoSource << "\"" << endl;
        return -1;
      }
      int vidLength = vid.get( CV_CAP_PROP_FRAME_COUNT );



      double reprojError;
        int totalPoints = 0;

      if( opts.intervalFrames > 0 ) {
        for( int i = opts.seekTo; i < vidLength; i += opts.intervalFrames ) {
          Detection *detection = db.load( i );
          if( detection ) {
            totalPoints += computeReprojectionError( *dist, *detection, reprojError );
          }
        }
      } else {

        Detection *detection = NULL;
        //while( (detection = db.loadAdvanceCursor()) != NULL ) {
        //  totalPoints += computeReprojectionError( *dist, *detection, reprojError );
        //}

      }


      if( totalPoints > 0 ) {
        reprojError /= totalPoints;
      }

      cout << "For " << totalPoints << " points from " << " images, mean reproj error is " << reprojError << endl;

      //
      //
      //      if( opts.intervalSeconds > 0 ) opts.intervalFrames = opts.intervalSeconds * vid.get( CV_CAP_PROP_FPS );
      //
      //      Mat img;
      //      while( vid.read( img ) ) {
      //        int currentFrame = vid.get( CV_CAP_PROP_POS_FRAMES );
      //
      //        Detection *detection = NULL;
      //        cout << "Extracting from " << currentFrame << ". ";
      //        if( !db.has( currentFrame ) || opts.doRewrite ) {
      //
      //          Mat grey;
      //          cvtColor( img, grey, CV_BGR2GRAY );
      //          vector<Point2f> pointbuf;
      //          detection = board->detectPattern( grey, pointbuf );
      //          cout << detection->size() << " features" << endl;
      //
      //          if( !db.save( currentFrame, *detection) ) {
      //            cerr << "set error: " << db.error().name() << endl;
      //          }
      //        } else {
      //          cout << "Data exists, skipping..." << endl;
      //
      //          detection = db.load( currentFrame );
      //        }
      //
      //
      //        if( opts.doDisplay ) {
      //          if( detection ) detection->drawCorners( *board, img );
      //
      //          imshow("Extract",img );
      //          waitKey( opts.waitKey );
      //        }
      //
      //        delete detection;
      //
      //        if( opts.intervalFrames > 1 ) {
      //          int destFrame = currentFrame + opts.intervalFrames - 1;
      //          if( destFrame < vidLength ) {
      //            vid.set( CV_CAP_PROP_POS_FRAMES, currentFrame + opts.intervalFrames - 1 );
      //          } else {
      //            // Done
      //            break;
      //          }
      //        }
      //      }

      return 0;


    }

    int computeReprojectionError( const DistortionModel &dist, const Detection det, double &reproj )
    {

      if( !(det.hasRot && det.hasTrans) ) return 0;

      ImagePointsVec reprojPts;
      dist.projectPoints( det.corners, det.rot, det.trans, reprojPts );

      double err = norm(Mat(det.points), Mat(reprojPts), CV_L2);

      reproj += err*err;
      return det.size();
    }



  private:
    CalcOpts opts;

    DetectionDb db;
    Board *board;

};



int main( int argc, char **argv ) 
{

  CalcOpts opts;
  string msg;
  if( !opts.parseOpts( argc, argv, msg ) ) {
    cout << msg << endl;
    exit(-1);
  }

  CalculateReprojectionMain main( opts );

  exit( main.run() );

}


