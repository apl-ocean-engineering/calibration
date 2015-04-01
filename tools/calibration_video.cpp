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
using namespace AplCam;

#include "video_splitters/video_splitter_opts.h"
#include "video_splitters/video_splitters.h"
using namespace AplCam::VideoSplitters;



using namespace cv;
using namespace std;

using kyotocabinet::HashDB;
using kyotocabinet::DB;

class CalibrationOpts {

  public:

    typedef enum { ANGULAR_POLYNOMIAL, RADIAL_POLYNOMIAL } CalibrationType_t;

    typedef enum { SPLIT_ALL, SPLIT_RANDOM, SPLIT_INTERVAL, SPLIT_NONE = -1 } SplitterType_t;

    CalibrationOpts()
      : dataDir("data"),
      boardName(), cameraName(),
      calibFlags(0), 
      videoFile(),
      ignoreCache( false ), saveBoardPoses( false ), fixSkew( false ),
      calibType( ANGULAR_POLYNOMIAL )
  {;}

    bool validate( string &msg)
    {
      if( cameraName.empty() ) { msg = "Camera name not set"; return false; }

      return true;
    }

    string dataDir;
    string boardName;
    string cameraName;
    int seekTo, intervalFrames;
    int calibFlags, randomize;
    string videoFile, resultsFile;
    bool ignoreCache, retryUnregistered, saveBoardPoses, fixSkew;
    CalibrationType_t calibType;
    SplitterType_t splitter;

    IntervalSplitterOpts intervalSplitterOpts;
    RandomSplitterOpts randomSplitterOpts;

    const string boardPath( void )
    { return dataDir + "/boards/" + boardName + ".yml"; }

    const string cachePath( void )
    { return dataDir + "/cache"; }

    const string imageCache( const Image &image )
    { return cachePath() + "/" + image.hash() + ".yml"; }

    const string tmpPath( const string &file )
    { return dataDir + "/tmp/" + file; }

    const string cameraPath( const string &filename )
    {
      string camDir(  dataDir + "/cameras/" + cameraName + "/" );
      if( !directory_exists( camDir ) ) mkdir_p( camDir );
      return camDir + filename;
    }



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
        { "ignore-cache", false, NULL, 'R' },
        { "fix-skew", false, NULL, 'k'},
        { "retry-unregistered", false, NULL, 'r' },
        { "save-board-poses", no_argument, NULL, 'S' },
        { "results-file", required_argument, NULL, 'Z' },
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
      while( (optVal = getopt_long( argc, argv, "Z:RSrb:c:d:km:?", long_options, &indexPtr )) != -1 ) {
        switch( optVal ) {
          case 'Z':
            resultsFile = optarg;
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
          case 'R':
            ignoreCache = true;
            break;
          case 'S':
            saveBoardPoses = true;
            break;
          case 'k':
            calibFlags |= PinholeCamera::CALIB_FIX_SKEW;
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
          case 'r':
            retryUnregistered = true;
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
      char *verb = argv[ optind ];
      bool success = false;
      if( strcasecmp( verb, "all" ) ) {
        splitter = SPLIT_ALL;
      } else if( strcasecmp( verb, "random" ) ) {
        splitter = SPLIT_RANDOM;
        success = randomSplitterOpts.parseOpts( argc, argv, msg );
      } else if( strcasecmp( verb, "interval" ) ) {
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

      if( !validate( msg ) ) {
        return false;
      }

      return true;
    }


};


class ResultsFile {
  public:
    ResultsFile( const string &filename )
      : _db()
    {
      _isOpen = _db.open( filename );
    }

    kyotocabinet::BasicDB::Error error( void ) { return _db.error(); }
    bool isOpened( void ) const { return _isOpen; }

    HashDB _db;
    bool _isOpen;
};


static double computeReprojectionErrors(
    const DistortionModel *dist,
    const Distortion::ObjectPointsVecVec &objectPoints,
    const Distortion::ImagePointsVecVec &imagePoints,
    const Distortion::RotVec &rvecs, 
    const Distortion::TransVec &tvecs,
    vector<float>& perViewErrors )
{
  ImagePointsVec reprojImgPoints;
  int i, totalPoints = 0;
  double totalErr = 0, err;
  perViewErrors.resize(objectPoints.size());

  for( i = 0; i < (int)objectPoints.size(); i++ )
  {
    if( objectPoints[i].size() > 0 ) {
      dist->projectPoints( Mat( objectPoints[i] ), rvecs[i], tvecs[i], reprojImgPoints );

      err = norm(Mat(imagePoints[i]), Mat(reprojImgPoints), CV_L2);
      int n = (int)objectPoints[i].size();
      perViewErrors[i] = (float)std::sqrt(err*err/n);
      totalErr += err*err;
      totalPoints += n;
    }
  }

  return std::sqrt(totalErr/totalPoints);
}

static void saveCameraParams( const string& filename,
    Size imageSize, const Board &board,
    const vector< Image > &imagesUsed,
    float aspectRatio, int flags,
    const DistortionModel *model, 
    const vector<Vec3d>& rvecs, const vector<Vec3d>& tvecs,
    const vector<float>& reprojErrs,
    const Distortion::ImagePointsVecVec &imagePoints,
    double totalAvgErr )
{

  FileStorage out( filename, FileStorage::WRITE );

  time_t tt;
  time( &tt );
  struct tm *t2 = localtime( &tt );
  char buf[1024];
  strftime( buf, sizeof(buf)-1, "%c", t2 );

  out << "calibration_time" << buf;

  if( !rvecs.empty() || !reprojErrs.empty() )
    out << "nframes" << (int)std::max(rvecs.size(), reprojErrs.size());
  out << "image_width" << imageSize.width;
  out << "image_height" << imageSize.height;
  out << "board_name" << board.name;
  out << "board_width" << board.size().width;
  out << "board_height" << board.size().height;
  out << "square_size" << board.squareSize;

  if( flags & CV_CALIB_FIX_ASPECT_RATIO )
    out << "aspectRatio" << aspectRatio;

  if( flags != 0 )
  {
    sprintf( buf, "flags: %s%s%s%s",
        flags & CV_CALIB_USE_INTRINSIC_GUESS ? "+use_intrinsic_guess" : "",
        flags & CV_CALIB_FIX_ASPECT_RATIO ? "+fix_aspectRatio" : "",
        flags & CV_CALIB_FIX_PRINCIPAL_POINT ? "+fix_principal_point" : "",
        flags & CV_CALIB_ZERO_TANGENT_DIST ? "+zero_tangent_dist" : "" );
    cvWriteComment( *out, buf, 0 );
  }

  out << "flags" << flags;

  model->write( out );

  out << "avg_reprojection_error" << totalAvgErr;
  if( !reprojErrs.empty() )
    out << "per_view_reprojection_errors" << Mat(reprojErrs);

  //  if( !rvecs.empty() && !tvecs.empty() )
  //  {
  //    CV_Assert(rvecs[0].type() == tvecs[0].type());
  //    Mat bigmat((int)rvecs.size(), 6, rvecs[0].type());
  //    for( int i = 0; i < (int)rvecs.size(); i++ )
  //    {
  //      Mat r = bigmat(Range(i, i+1), Range(0,3));
  //      Mat t = bigmat(Range(i, i+1), Range(3,6));
  //
  //      CV_Assert(rvecs[i].rows == 3 && rvecs[i].cols == 1);
  //      CV_Assert(tvecs[i].rows == 3 && tvecs[i].cols == 1);
  //      //*.t() is MatExpr (not Mat) so we can use assignment operator
  //      r = rvecs[i].t();
  //      t = tvecs[i].t();
  //    }
  //    cvWriteComment( *out, "a set of 6-tuples (rotation vector + translation vector) for each view", 0 );
  //    out   << "extrinsic_parameters" << bigmat;
  //  }

  //  out << "images_used" << "[";
  //  for( vector<Image>::const_iterator img = imagesUsed.begin(); img < imagesUsed.end(); ++img ) {
  //    out << img->fileName();
  //  }
  //  out << "]";

  if( !imagePoints.empty() )
  {
    Mat imagePtMat((int)imagePoints.size(), (int)imagePoints[0].size(), CV_32FC2);
    for( int i = 0; i < (int)imagePoints.size(); i++ )
    {
      Mat r = imagePtMat.row(i).reshape(2, imagePtMat.cols);
      Mat imgpti(imagePoints[i]);
      imgpti.copyTo(r);
    }
    out << "image_points" << imagePtMat;
  }
}


static string mkCameraFileName( const string &cameraName)
{
  char strtime[32], buffer[80];
  time_t tt;
  time( &tt );
  struct tm *t2 = localtime( &tt );
  strftime( strtime, 32, "%y%m%d_%H%M%S", t2 );
  snprintf( buffer, 79, "%s_%s.yml", cameraName.c_str(), strtime );
  return  string( buffer );
}


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
  float aspectRatio = 1.f;

  Distortion::ImagePointsVecVec imagePoints;
  Distortion::ObjectPointsVecVec objectPoints;

  DetectionDb db;

  if( ! db.open( opts.cachePath(), opts.videoFile, 
        ( opts.saveBoardPoses == true ? true : false ) ) ) {
    cerr << "Open error: " << db.error().name() << endl;
    return -1;
  }

  string videoSource( opts.videoFile );
  VideoCapture vid( videoSource );
  if( !vid.isOpened() ) {
    cerr << "Couldn't open video source \"" << videoSource << "\"" << endl;
    return -1;
  }
  int vidLength = vid.get( CV_CAP_PROP_FRAME_COUNT );

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


  cout << "Have detection set with " << detSet.size() << " points" << endl;


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

  double rms = distModel->calibrate( objectPoints, imagePoints, 
      imageSize, rvecs, tvecs, flags );

  //  ///*|CV_CALIB_FIX_K3*/|CV_CALIB_FIX_K4|CV_CALIB_FIX_K5);
  printf("RMS error reported by calibrateCamera: %g\n", rms);

  //  bool ok = checkRange(cameraMatrix) && checkRange(distCoeffs);

  bool ok = true;

  vector<float> reprojErrs;
  double totalAvgErr = 0;
  totalAvgErr = computeReprojectionErrors(distModel, objectPoints, imagePoints, rvecs, tvecs, reprojErrs );

  if( ok ) {
    string cameraFile( opts.cameraPath(mkCameraFileName( opts.cameraName ) ) );
    cout << "Writing results to " << cameraFile << endl;

    vector<Image> imagesUsed;
    bool writeExtrinsics = false,
         writePoints = false;

    saveCameraParams( cameraFile, imageSize,
        *board, imagesUsed, aspectRatio,
        flags, distModel,
        writeExtrinsics ? rvecs : vector<Vec3d>(),
        writeExtrinsics ? tvecs : vector<Vec3d>(),
        writeExtrinsics ? reprojErrs : vector<float>(),
        writePoints ? imagePoints : Distortion::ImagePointsVecVec(),
        totalAvgErr );

    if( opts.saveBoardPoses ) {
      for( int i = 0; i < detSet.size(); ++i ) {
        Detection &det( detSet[i] );
        det.rot = rvecs[i];
        det.trans = tvecs[i];

        if( ! db.update( detSet[i].frame, det ) )
          cerr << "Trouble saving updated poses: " << db.error().name() << endl;
      }
    }
  }

  printf("%s. avg reprojection error = %.2f\n",
      ok ? "Calibration succeeded" : "Calibration failed",
      totalAvgErr);

  delete distModel;
  delete board;

  return 0;
}
