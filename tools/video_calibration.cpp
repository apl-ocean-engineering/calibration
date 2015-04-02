#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <cctype>
#include <stdio.h>
#include <string.h>
#include <strings.h>
#include <time.h>
#include <getopt.h>

#include <iostream>

#include "my_undistort.h"

#include "distortion_model.h"
#include "distortion_angular_polynomial.h"
#include "distortion_radial_polynomial.h"
using namespace Distortion;

#include "file_utils.h"
#include "board.h"
#include "detection.h"
#include "detection_set.h"
#include "image.h"

#include "calibration_db.h"
#include "calibration_opts_common.h"
using namespace AplCam;

#include "video_splitters/video_splitter_opts.h"
#include "video_splitters/video_splitters.h"
using namespace AplCam::VideoSplitters;



using namespace cv;
using namespace std;

using kyotocabinet::HashDB;
using kyotocabinet::DB;

class CalibrationOpts : public AplCam::CalibrationOptsCommon {

  public:

    typedef enum { SPLIT_ALL, SPLIT_RANDOM, SPLIT_INTERVAL, SPLIT_NONE = -1 } SplitterType_t;

    CalibrationOpts()
      : CalibrationOptsCommon(), 
      calibrationDb(),
      videoFile(),
      saveBoardPoses( false ), fixSkew( false ), overwriteDb( false )
  {;}

    string calibrationDb;

    string videoFile;

    bool saveBoardPoses, fixSkew, overwriteDb;

    CalibrationType_t calibType;
    SplitterType_t splitter;

    IntervalSplitterOpts intervalSplitterOpts;
    RandomSplitterOpts randomSplitterOpts;


    //== Option parsing and help ==
    void help()
    {
      printf( "This is a camera calibration sample.\n"
          "Usage: calibration\n"
          "     -d <data directory>      # Specify top-level directory for board/camera/cache files.\n"
          "     --board,-b <board_name>    # Name of calibration pattern\n"
          "     --camera, -c <camera_name> # Name of camera\n"
          "     --ignore-cache, -i       # Ignore and overwrite files in cache\n"
          "     --retry-unregistered, -r   # Re-try to find the chessboard if the cache file is empty\n"
          "     --calibration-model, -m   # Set the distortion model to: angular, radial, radial8\n"
          "     --fix-skew, -k            # Fix skew (alpha) to 0\n"
          //     "     [-d <delay>]             # a minimum delay in ms between subsequent attempts to capture a next view\n"
          //     "                              # (used only for video capturing)\n"
          //     "     [-o <out_camera_params>] # the output filename for intrinsic [and extrinsic] parameters\n"
          //     "     [-op]                    # write detected feature points\n"
          //     "     [-oe]                    # write extrinsic parameters\n"
          //     "     [-zt]                    # assume zero tangential distortion\n"
          //     "     [-a <aspectRatio>]      # fix aspect ratio (fx/fy)\n"
          //     "     [-p]                     # fix the principal point at the center\n"
          //     "     [-v]                     # flip the captured images around the horizontal axis\n"
          //     "     [-V]                     # use a video file, and not an image list, uses\n"
          //     "                              # [input_data] string for the video file name\n"
          //     "     [-su]                    # show undistorted images after calibration\n"
        "     [input_data]             # list of files to use\n"
        "\n" );
      //printf("\n%s",usage);
      //printf( "\n%s", liveCaptureHelp );
    }


    bool parseOpts( int argc, char **argv, string &msg )
    {
      stringstream msgstrm;

      static struct option long_options[] = {
        { "data-directory", true, NULL, 'd' },
        { "board", true, NULL, 'b' },
        { "camera", true, NULL, 'c' },
        { "calibation-model", true, NULL, 'm' },
        { "fix-skew", false, NULL, 'k'},
        { "save-board-poses", no_argument, NULL, 'S' },
        { "calibration-file", required_argument, NULL, 'z' },
        { "calibration-db", required_argument, NULL, 'Z' },
        { "overwrite-db", no_argument, NULL, 'y' },
        { "help", false, NULL, '?' },
        { 0, 0, 0, 0 }
      };


      if( argc < 2 )
      {
        help();
        return false;
      }

      int indexPtr;
      int optVal;
      string c;

      // The '+' option ensures it stops on the first non-conforming option. Required for the
      //   cmd opt1 opt2 opt3 verb verb_opt1 files ...
      // pattern I'm using
      while( (optVal = getopt_long( argc, argv, "+z:yZ:RSrb:c:d:km:?", long_options, &indexPtr )) != -1 ) {
        switch( optVal ) {
          case 'z':
            calibrationFile = optarg;
            break;
          case 'Z':
            calibrationDb = optarg;
            break;
          case 'd':
            dataDir = optarg;
            break;
          case 'b':
            boardName = optarg;
            break;
          case 'c':
            cameraName = optarg;
            break;
          case 'S':
            saveBoardPoses = true;
            break;
          case 'k':
            calibFlags |= PinholeCamera::CALIB_FIX_SKEW;
            break;
          case 'y':
            overwriteDb = true;
            break;
          case 'm':
            c = optarg;
            if( c.compare("angular") == 0 ) {
              calibType = ANGULAR_POLYNOMIAL;
            } else if (c.compare("radial") == 0) {
              calibType = RADIAL_POLYNOMIAL;
            } else if (c.compare("radial8") == 0) {
              calibType = RADIAL_POLYNOMIAL;
              calibFlags |= CV_CALIB_RATIONAL_MODEL;
            } else {
              cerr <<  "Can't figure out the calibration model \"" <<  c << "\"";
              return false;
            }
            break;
          case '?': 
            help();
            break;
          default:
            return false;

        }
      }

      if( optind >= argc ) {
        msg = "Must specify splitter type on command line";
        return false;
      }

      // Next, expect a verb
      char *verb = argv[ optind++ ];
      bool success = false;
      if( !strcasecmp( verb, "all" ) ) {
        splitter = SPLIT_ALL;
        success = true;
      } else if( !strcasecmp( verb, "random" ) ) {
        splitter = SPLIT_RANDOM;
        success = randomSplitterOpts.parseOpts( argc, argv, msg );
      } else if( !strcasecmp( verb, "interval" ) ) {
        splitter = SPLIT_INTERVAL;
        success = intervalSplitterOpts.parseOpts( argc, argv, msg );
      } else {
        msgstrm << "Don't understand verb \"" << verb << "\"";
        msg = msgstrm.str();
        return false;
      }

      if( success == false ) return success;


      if( optind >= argc ) {
        msg = "Must specify video file on command line";
        return false;
      }

      videoFile = argv[ optind ];

      if( !file_exists( videoFile ) ) {
        cerr << "Can't open video file " << videoFile << endl;
        return false;
      }

      if( !validate( msg ) ) return false;

      return true;
    }


    virtual bool validate( string &msg )
    {
      if( !calibrationDb.empty() ) {
        if( !calibrationFile.empty() ) {
          msg = "Can't set both calibration file and calibration db";
          return false;
        }
      }

      if( !CalibrationOptsCommon::validate( msg ) ) return false;

      // The super will auto-fill calibrationFile if not set
      if( !calibrationDb.empty() ) calibrationFile.clear();

      return true;
    }

};


int main( int argc, char** argv )
{

  CalibrationOpts opts;

  string optsMsg;
  if( !opts.parseOpts( argc, argv, optsMsg ) ) {
    cout << optsMsg << endl;
    exit(-1);
  }

  Board *board = Board::load( opts.boardPath(), opts.boardName );

  Size imageSize;
  //float aspectRatio = 1.f;

  Distortion::ImagePointsVecVec imagePoints;
  Distortion::ObjectPointsVecVec objectPoints;

  DetectionDb db;

  if( ! db.open( opts.cachePath(), opts.videoFile, 
        ( opts.saveBoardPoses == true ? true : false ) ) ) {
    cerr << "Error opening db error: " << db.error().name() << endl;
    return -1;
  }

  string videoSource( opts.videoFile );
  VideoCapture vid( videoSource );
  if( !vid.isOpened() ) {
    cerr << "Couldn't open video source \"" << videoSource << "\"" << endl;
    return -1;
  }
  //int vidLength = vid.get( CV_CAP_PROP_FRAME_COUNT );

  // Get image size
  imageSize = Size( vid.get( CV_CAP_PROP_FRAME_WIDTH ), vid.get(CV_CAP_PROP_FRAME_HEIGHT ) );

  DetectionSet detSet;
  switch( opts.splitter ) {
    case CalibrationOpts::SPLIT_ALL:
      AllVideoSplitter().generate( db, detSet );
      break;
    case CalibrationOpts::SPLIT_RANDOM:
      RandomVideoSplitter( opts.randomSplitterOpts ).generate( db, detSet );
      break;
    case CalibrationOpts::SPLIT_INTERVAL:
      IntervalVideoSplitter( opts.intervalSplitterOpts ).generate( db, detSet );
      break;
    default:
      cerr << "Unknown video splitter." << endl;
      exit(-1);
  }


  cout << "Have detection set " << detSet.name() << " with " << detSet.size() << " points" << endl;

  int count = detSet.imageObjectPoints( imagePoints, objectPoints );


  cout << "Using " << count << " points from " << imagePoints.size() << " images" << endl;

  if( imagePoints.size() < 3 ) {
    cerr << "Not enough images.  Stopping." << endl;
    exit(-1);
  }

  vector< Vec3d > rvecs, tvecs;

  int flags =  opts.calibFlags;

  DistortionModel *distModel = NULL;
  switch( opts.calibType ) {
    case CalibrationOpts::ANGULAR_POLYNOMIAL:
      distModel = new Distortion::AngularPolynomial;
      break;
    case CalibrationOpts::RADIAL_POLYNOMIAL:
      distModel = new Distortion::RadialPolynomial;
      break;
  }

  if( !distModel ) {
    cerr << "Something went wrong choosing a distortion model." << endl;
    exit(-1);
  }

  CalibrationResult result;
  distModel->calibrate( objectPoints, imagePoints, 
      imageSize, result, flags );

  if( result.success ) {
    //  ///*|CV_CALIB_FIX_K3*/|CV_CALIB_FIX_K4|CV_CALIB_FIX_K5);
    cout << "RMS error reported by calibrateCamera: " << result.rms << endl;
    cout << "Residual reported by calibrateCamera: " << result.residual << endl;

    //  bool ok = checkRange(cameraMatrix) && checkRange(distCoeffs);

//    vector<float> reprojErrs;
//
//    string cameraFile( opts.cameraPath(mkCameraFileName( opts.cameraName ) ) );
//    cout << "Writing results to " << cameraFile << endl;
//
//    vector<Image> imagesUsed;
//    bool writeExtrinsics = false,
//         writePoints = false;
//

    CalibrationSerializer ser;

    ser.setCamera( distModel )
       .setResult( &result )
       .setBoard( board );

    if( !opts.calibrationDb.empty() ) {
      cout << "Writing to calibration db " << opts.calibrationDb << endl;

      CalibrationDb db( opts.calibrationDb );
      if( db.has( detSet.name() ) && !opts.overwriteDb ) {
        cerr << "Already have result in db " << opts.calibrationDb << " with key " << detSet.name() << endl;
      } else {
        db.save( detSet.name(), ser );
      }

    } else if( !opts.calibrationFile.empty() ) {
      cout << "Writing calibration to " << opts.calibrationFile << endl;
      if( !ser.writeFile( opts.calibrationFile ) ) {
        cerr << "Error writing to opts.calibrationFile" << endl;;
      }
    }

//    saveCameraParams( cameraFile, imageSize,
//        *board, imagesUsed, aspectRatio,
//        flags, distModel,
//        writeExtrinsics ? result.rvecs : vector<Vec3d>(),
//        writeExtrinsics ? result.tvecs : vector<Vec3d>(),
//        writeExtrinsics ? reprojErrs : vector<float>(),
//        writePoints ? imagePoints : Distortion::ImagePointsVecVec(),
//        result.rms );

    if( opts.saveBoardPoses ) {
      cout << "Writing estimate board poses back to database." << endl;
      for( size_t i = 0; i < detSet.size(); ++i ) {
        Detection &det( detSet[i] );
        det.rot = result.rvecs[i];
        det.trans = result.tvecs[i];

        if( ! db.update( detSet[i].frame, det ) )
          cerr << "Trouble saving updated poses: " << db.error().name() << endl;
      }
    }

  } else {
    cout << "Calibration failed." << endl;
  }

  //printf("%s. avg reprojection error = %.2f\n",
  //    ok ? "Calibration succeeded" : "Calibration failed",
  //    totalAvgErr);

  delete distModel;
  delete board;

  return 0;
}
