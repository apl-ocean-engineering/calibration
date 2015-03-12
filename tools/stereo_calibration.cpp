#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <iostream>
#include <algorithm>
#include <vector>

#include <boost/filesystem.hpp>

#include <cctype>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <getopt.h>


#include "file_utils.h"
#include "board.h"
#include "detection.h"
#include "image.h"

using namespace cv;
using namespace std;

namespace fs = boost::filesystem;




class StereoCalibrationOpts {
  public:
    StereoCalibrationOpts( int argc, char **argv )
      : dataDir("data"),
      boardName(), 
      _pairName(),    // Private as it has a bespoke reader function
      inFiles(),
      ignoreCache( false )
  {
    string msg;
    if( !parseOpts( argc, argv, msg ) ) {
      cerr << msg << endl;
      exit(-1);
    }
  }

    bool validate( string &msg)
    {
      if( boardName.empty() ) { msg = "Board name not set"; return false; }
      if( cameraName[0].empty() || cameraName[1].empty() ) { msg = "Camera name not set"; return false; }

      for( int i = 0; i < 2; ++i ) {
        if( !fs::is_directory( cameraPath(i) ) ) { 
          stringstream strm;
          strm << "Can't find camera directory " << cameraPath(i);
          msg = strm.str();

          return false;
        }
      }

      return true;
    }

    string dataDir;
    string boardName;
    string cameraName[2];
    vector< string > inFiles;
    bool ignoreCache, retryUnregistered;

    const string pairName( void ) const 
    {
      if( _pairName.empty() ) 
      {
        return cameraName[0] + "--" + cameraName[1];
      } 

      return _pairName;
    }

    const string boardPath( void )
    { return dataDir + "/boards/" + boardName + ".yml"; }

    const string cachePath( void )
    { return dataDir + "/cache"; }

    const string imageCache( const string &image )
    { return cachePath() + "/" + image + ".yml"; }

    const string tmpPath( const string &file )
    { return dataDir + "/tmp/" + file; }

    const string cameraPath( int which, const string &filename = String("") )
    {
      string camDir(  dataDir + "/cameras/" + cameraName[which] + "/" );
      if( !directory_exists( camDir ) ) fs::create_directories( camDir );
      return camDir + filename;
    }

    static string SliceFilename( const fs::directory_entry &entry )
    { return entry.path().filename().string(); }
    static bool NotYaml( const string &name )
    { return name.find(".yml") == string::npos; }

    const string cameraLatest( int which )
    {
      string camDir(  dataDir + "/cameras/" + cameraName[which] + "/" );

      vector< string > filenames;
      std::transform( fs::directory_iterator( camDir ), fs::directory_iterator(), 
          std::back_inserter( filenames ), SliceFilename );
      std::remove_if( filenames.begin(), filenames.end(), NotYaml );
      std::sort( filenames.begin(), filenames.end() );

      return cameraPath( which, filenames.back() );
    }

    const string stereoPairPath( const string &filename )
    {
      string pairDir(  dataDir + "/stereo_pairs/" + pairName() + "/" );
      if( !directory_exists( pairDir ) ) fs::create_directories( pairDir );
      return pairDir + filename;
    }



    //== Option parsing and help ==
    void help()
    {
      printf( "This is a camera calibration sample.\n"
          "Usage: calibration\n"
          "     -d <data directory>      # Specify top-level directory for board/camera/cache files.\n"
          "     --board,-b <board_name>    # Name of calibration pattern\n"
          "     --camera, -c camera_name1:camera_name2 # Name of camera\n"
          "     --pair, -p <pair_name>          # Name of stereo pair (set automatically if not specified)\n"
          "     --ignore-cache, -i       # Ignore and overwrite files in cache\n"
          "     --retry-unregistered, -r   # Re-try to find the chessboard if the cache file is empty\n"
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
      static struct option long_options[] = {
        { "data_directory", required_argument, NULL, 'd' },
        { "board", required_argument, NULL, 'b' },
        { "camera", required_argument, NULL, 'c' },
        { "pair", required_argument, NULL, 'p' },
        { "ignore-cache", false, NULL, 'i' },
        { "retry-unregistered", false, NULL, 'r' },
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
      string names;
      stringstream strm;
      size_t pos = string::npos;

      while( (optVal = getopt_long( argc, argv, "irb:c:d:p:?", long_options, &indexPtr )) != -1 ) {
        switch( optVal ) {
          case 'b':
            boardName = optarg;
            break;
          case 'c':
            // Split the optarg
            names = optarg;
            pos = names.find_first_of( ":," );
            if( pos == string::npos ) {
              msg = "Couldn't parse camera names";
              return false;
            }

            cameraName[0].assign( names, 0, pos );
            cameraName[1].assign( names, pos+1, string::npos );
            break;
          case 'd':
            dataDir = optarg;
            break;
          case 'i':
            ignoreCache = true;
            break;
          case 'p':
            _pairName = optarg;
            break;
          case 'r':
            retryUnregistered = true;
            break;
          case '?': 
            help();
            exit(0);
            break;
          default:
            strm << "Unknown option \"" << optopt  << "\"";
            msg = strm.str();
            return false;
        }
      }

      if( optind == argc )
      {
        cout << "No input files specified." << endl;
        exit(-1);
      }

      for( int i = optind; i < argc; ++i ) {
        string infile( argv[i] );

        if( !fs::is_regular_file( infile ) ) {
          cout << "Couldn't open input file \"" << infile << "\"" << endl;
          exit(-1);
        }

        inFiles.push_back( infile );
      }

      //    for( i = 1; i < argc; i++ )
      //    {
      //        const char* s = argv[i];
      //        if( strcmp( s, "-w" ) == 0 )
      //        {
      //            if( sscanf( argv[++i], "%u", &boardSize.width ) != 1 || boardSize.width <= 0 )
      //                return fprintf( stderr, "Invalid board width\n" ), -1;
      //        }
      //        else if( strcmp( s, "-h" ) == 0 )
      //        {
      //            if( sscanf( argv[++i], "%u", &boardSize.height ) != 1 || boardSize.height <= 0 )
      //                return fprintf( stderr, "Invalid board height\n" ), -1;
      //        }
      //        else if( strcmp( s, "-pt" ) == 0 )
      //        {
      //            i++;
      //            if( !strcmp( argv[i], "circles" ) )
      //                pattern = CIRCLES_GRID;
      //            else if( !strcmp( argv[i], "acircles" ) )
      //                pattern = ASYMMETRIC_CIRCLES_GRID;
      //            else if( !strcmp( argv[i], "chessboard" ) )
      //                pattern = CHESSBOARD;
      //            else
      //                return fprintf( stderr, "Invalid pattern type: must be chessboard or circles\n" ), -1;
      //        }
      //        else if( strcmp( s, "-s" ) == 0 )
      //        {
      //            if( sscanf( argv[++i], "%f", &squareSize ) != 1 || squareSize <= 0 )
      //                return fprintf( stderr, "Invalid board square width\n" ), -1;
      //        }
      //        ..else if( strcmp( s, "-n" ) == 0 )
      //        {
      //            if( sscanf( argv[++i], "%u", &nframes ) != 1 || nframes <= 3 )
      //                return printf("Invalid number of images\n" ), -1;
      //        }
      //        else if( strcmp( s, "-a" ) == 0 )
      //        {
      //            if( sscanf( argv[++i], "%f", &aspectRatio ) != 1 || aspectRatio <= 0 )
      //                return printf("Invalid aspect ratio\n" ), -1;
      //            flags |= CV_CALIB_FIX_ASPECT_RATIO;
      //        }
      //        else if( strcmp( s, "-d" ) == 0 )
      //        {
      //            if( sscanf( argv[++i], "%u", &delay ) != 1 || delay <= 0 )
      //                return printf("Invalid delay\n" ), -1;
      //        }
      //        else if( strcmp( s, "-op" ) == 0 )
      //        {
      //            writePoints = true;
      //        }
      //        else if( strcmp( s, "-oe" ) == 0 )
      //        {
      //            writeExtrinsics = true;
      //        }
      //        else if( strcmp( s, "-zt" ) == 0 )
      //        {
      //            flags |= CV_CALIB_ZERO_TANGENT_DIST;
      //        }
      //        else if( strcmp( s, "-p" ) == 0 )
      //        {
      //            flags |= CV_CALIB_FIX_PRINCIPAL_POINT;
      //        }
      //        else if( strcmp( s, "-v" ) == 0 )
      //        {
      //            flipVertical = true;
      //        }
      //        else if( strcmp( s, "-V" ) == 0 )
      //        {
      //            videofile = true;
      //        }
      //        else if( strcmp( s, "-o" ) == 0 )
      //        {
      //            outputFilename = argv[++i];
      //        }
      //        else if( strcmp( s, "-su" ) == 0 )
      //        {
      //            showUndistorted = true;
      //        }
      //        else if( s[0] != '-' )
      //        {
      //            if( isdigit(s[0]) )
      //                sscanf(s, "%d", &cameraId);
      //            else
      //                inputFilename = s;
      //        }
      //        else
      //            return fprintf( stderr, "Unknown option %s", s ), -1;
      //    }

      return validate(msg);
    }


  private:

    string _pairName;

};

static double computeReprojectionErrors(
    const vector<vector<Point3f> >& objectPoints,
    const vector<vector<Point2f> >& imagePoints,
    const vector<Mat>& rvecs, const vector<Mat>& tvecs,
    const Mat& cameraMatrix, const Mat& distCoeffs,
    vector<float>& perViewErrors )
{
  vector<Point2f> imagePoints2;
  int i, totalPoints = 0;
  double totalErr = 0, err;
  perViewErrors.resize(objectPoints.size());

  for( i = 0; i < (int)objectPoints.size(); i++ )
  {
    projectPoints(Mat(objectPoints[i]), rvecs[i], tvecs[i],
        cameraMatrix, distCoeffs, imagePoints2);
    err = norm(Mat(imagePoints[i]), Mat(imagePoints2), CV_L2);
    int n = (int)objectPoints[i].size();
    perViewErrors[i] = (float)std::sqrt(err*err/n);
    totalErr += err*err;
    totalPoints += n;
  }

  return std::sqrt(totalErr/totalPoints);
}

static bool runCalibration( vector<vector<Point2f> > imagePoints,
    vector< vector<Point3f> > objectPoints,
    Size imageSize, 
    float aspectRatio,
    int flags, Mat& cameraMatrix, Mat& distCoeffs,
    vector<Mat>& rvecs, vector<Mat>& tvecs,
    vector<float>& reprojErrs,
    double& totalAvgErr)
{
  cameraMatrix = Mat::eye(3, 3, CV_64F);
  if( flags & CV_CALIB_FIX_ASPECT_RATIO )
    cameraMatrix.at<double>(0,0) = aspectRatio;

  distCoeffs = Mat::zeros(8, 1, CV_64F);

  double rms = calibrateCamera(objectPoints, imagePoints, imageSize, cameraMatrix,
      distCoeffs, rvecs, tvecs, flags|CV_CALIB_RATIONAL_MODEL);

  ///*|CV_CALIB_FIX_K3*/|CV_CALIB_FIX_K4|CV_CALIB_FIX_K5);
  printf("RMS error reported by calibrateCamera: %g\n", rms);

  bool ok = checkRange(cameraMatrix) && checkRange(distCoeffs);

  totalAvgErr = computeReprojectionErrors(objectPoints, imagePoints,
      rvecs, tvecs, cameraMatrix, distCoeffs, reprojErrs);

  return ok;
}


static void saveStereoPairParams( const string& filename,
    Size imageSize, const Board &board,
    const vector< Image > &imagesUsed,
    float aspectRatio, int flags,
    const Mat& cameraMatrix, const Mat& distCoeffs,
    const vector<Mat>& rvecs, const vector<Mat>& tvecs,
    const vector<float>& reprojErrs,
    const vector<vector<Point2f> >& imagePoints,
    double totalAvgErr )
{

  FileStorage out( filename, FileStorage::WRITE );

  time_t tt;
  time( &tt );
  struct tm *t2 = localtime( &tt );
  char buf[1024];
  strftime( buf, sizeof(buf)-1, "%c", t2 );

  out << "calibration_time" << buf;
  //
  //  if( !rvecs.empty() || !reprojErrs.empty() )
  //    out << "nframes" << (int)std::max(rvecs.size(), reprojErrs.size());
  //  out << "image_width" << imageSize.width;
  //  out << "image_height" << imageSize.height;
  //  out << "board_name" << board.name;
  //  out << "board_width" << board.size().width;
  //  out << "board_height" << board.size().height;
  //  out << "square_size" << board.squareSize;
  //
  //  if( flags & CV_CALIB_FIX_ASPECT_RATIO )
  //    out << "aspectRatio" << aspectRatio;
  //
  //  if( flags != 0 )
  //  {
  //    sprintf( buf, "flags: %s%s%s%s",
  //        flags & CV_CALIB_USE_INTRINSIC_GUESS ? "+use_intrinsic_guess" : "",
  //        flags & CV_CALIB_FIX_ASPECT_RATIO ? "+fix_aspectRatio" : "",
  //        flags & CV_CALIB_FIX_PRINCIPAL_POINT ? "+fix_principal_point" : "",
  //        flags & CV_CALIB_ZERO_TANGENT_DIST ? "+zero_tangent_dist" : "" );
  //    cvWriteComment( *out, buf, 0 );
  //  }
  //
  //  out << "flags" << flags;
  //
  //  out << "camera_matrix" << cameraMatrix;
  //  out << "distortion_coefficients" << distCoeffs;
  //
  //  out << "avg_reprojection_error" << totalAvgErr;
  //  if( !reprojErrs.empty() )
  //    out << "per_view_reprojection_errors" << Mat(reprojErrs);
  //
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
  //
  //  out << "images_used" << "[";
  //  for( vector<Image>::const_iterator img = imagesUsed.begin(); img < imagesUsed.end(); ++img ) {
  //    out << img->fileName();
  //  }
  //  out << "]";
  //
  //  if( !imagePoints.empty() )
  //  {
  //    Mat imagePtMat((int)imagePoints.size(), (int)imagePoints[0].size(), CV_32FC2);
  //    for( int i = 0; i < (int)imagePoints.size(); i++ )
  //    {
  //      Mat r = imagePtMat.row(i).reshape(2, imagePtMat.cols);
  //      Mat imgpti(imagePoints[i]);
  //      imgpti.copyTo(r);
  //    }
  //    out << "image_points" << imagePtMat;
  //  }
}

class Camera
{
  public:
    Camera( const string &name )
      : _name(name), _cam(), _dist()
    {; }

    const Mat &cameraMatrix( void ) const { return _cam; }
    const Mat &cam( void ) const { return _cam; }

    const Mat &distCoeffs( void ) const { return _dist; }

    bool loadCache( const string &cacheFile )
    {
      cout << "Loading camera file " << cacheFile << endl;

      if( !file_exists( cacheFile ) ) {
        cerr << "Cache  file \"" << cacheFile << "\" doesn't exist." << endl;
        return false;
      }

      FileStorage cache( cacheFile, FileStorage::READ );

      cache["camera_matrix"] >> _cam;
      cache["distortion_coefficients"] >> _dist;

      // Insert validation here
      //

      return true;
    }

  private:

    string _name, _cache;
    Mat _cam, _dist;
};



static string mkStereoPairFileName( void )
{
  char strtime[32];
  time_t tt;
  time( &tt );
  struct tm *t2 = localtime( &tt );
  strftime( strtime, 32, "cal_%y%m%d_%H%M%S.yml", t2 );
  return  string( strtime );
}


int main( int argc, char** argv )
{

  StereoCalibrationOpts opts( argc, argv );

  Board *board = Board::load( opts.boardPath(), opts.boardName );

  Size imageSize;
  float aspectRatio = 1.f;
  bool writeExtrinsics = false, writePoints = false;

  vector< vector<Point2f> > imagePoints[2];
  vector< vector<Point3f> > objectPoints;

  if( opts.inFiles.size() < 1 ) {
    cout << "No input files specified on command line." << endl;
    exit(-1);
  }

  // Load the camera files
  Camera cameras[2] = { Camera( opts.cameraName[0] ),
    Camera( opts.cameraName[1] ) };

  for( int i = 0; i < 2; ++i ) 
    if( !cameras[i].loadCache( opts.cameraLatest( i ) ) ) {
      cerr << "Couldn't load calibration for camera \"" << opts.cameraName[i] << endl;
      exit(-1);
    }

  vector<Image> imagesUsed;
  vector< pair< Image, Image > > images;

  if( opts.ignoreCache ) cout << "Ignoring cached data." << endl;

  for( int i = 0; i < opts.inFiles.size(); ++i ) {
    cout << "Processing " << i << " : " << opts.inFiles[i] << endl;
    Mat view, viewGray;

    view = imread(opts.inFiles[i], 1);

    //imageSize = view.size();

    // Break out two ROIs
    int roiWidth = view.size().width/2;
    Size roiSize( roiWidth, view.size().height );

    Mat rois[2] = { Mat( view, Rect( Point(0, 0), roiSize ) ),
      Mat( view, Rect( Point(roiWidth, 0), roiSize ) ) };

    Image img[2] = { Image( opts.inFiles[i] + "-left", rois[0] ),
      Image( opts.inFiles[i] + "-right", rois[1] ) };

    images.push_back( make_pair( img[0], img[1] ) );

    // Technically speaking you could eager load the images ... you don't even need them
    //  if you're reading from the cache.

    // Check for cached data
    bool doRegister = true;
    Detection *detection[2] = { NULL, NULL };

    for( int i = 0; i < 2; ++i ) {

      if( !opts.ignoreCache && (detection[i] = Detection::loadCache(  opts.imageCache( img[i].fileName() ) )) != NULL ) {
        doRegister = false;
        if( opts.retryUnregistered && detection[i] && (detection[i]->points.size() == 0) ) doRegister = true;
      }

      if( doRegister == false ) {
        cout << "  ... loaded data from cache." << endl;
      } else {

        cout << "  No cached data, searching for calibration pattern." << endl;

        //if( flipVertical )
        //  flip( view, view, 0 );

        vector<Point2f> pointbuf;
        cvtColor(rois[i], viewGray, COLOR_BGR2GRAY);

        detection[i] = board->detectPattern( viewGray, pointbuf );

        if( detection[i]->found )  
          cout << "  Found calibration pattern." << endl;

        detection[i]->writeCache( *board, opts.imageCache( img[i].fileName() ) );
      }
    }

    if( detection[0] != NULL || detection[1] != NULL ) {
      // Find the overlapping tags between each

      SharedPoints shared( AprilTagsDetection::sharedWith( *detection[0], *detection[1] ) );

      assert( (shared.worldPoints.size() != shared.aPoints.size()) ||
          (shared.worldPoints.size() == shared.bPoints.size() ) );

      if( shared.worldPoints.size() > 0 ) {
        objectPoints.push_back( shared.worldPoints );
        imagePoints[0].push_back( shared.aPoints );
        imagePoints[1].push_back( shared.bPoints );


    Mat canvas( std::max( images[i].first.size().height, images[i].second.size().height ),
        images[i].first.size().width + images[i].second.size().width,
        images[i].first.img().type() );
    Mat rectified[2] = {
      Mat( canvas, Rect( Point(0,0), images[i].first.size()) ),
      Mat( canvas, Rect(Point( images[i].first.size().width, 0 ), images[i].second.size() )) };

    images[i].first.img().copyTo( rectified[0] );
    images[i].second.img().copyTo( rectified[1] );

        for( int j = 0; j < shared.worldPoints.size(); ++j ) {
          //cout << shared.worldPoints[j] << shared.aPoints[j] << shared.bPoints[j] << endl;
          float i = (float)j / shared.worldPoints.size();
          Scalar color( 255*i, 0, (1-i)*255);

          cv::circle( rectified[0], shared.aPoints[j], 5, color, -1 );
          cv::circle( rectified[1], shared.bPoints[j], 5, color, -1 );

          cv::line(  canvas, shared.aPoints[j], shared.bPoints[j] + Point2f( images[i].first.size().width, 0 ),
              color, 1 );
        }

    string outfile( opts.tmpPath( String("annotated/") +  images[i].first.basename() + ".jpg" ) );
    mkdir_p( outfile );
    imwrite(  outfile, canvas );


      }

    }

    if( detection[0] ) delete detection[0];
    if( detection[1] ) delete detection[1];
  } // For  each image

  cout << "Calibrating stereo pair with " << objectPoints.size() << " sets of points." << endl;

  double reprojError;
  Mat r, t, e, f;

  // Local copies as they might be changed in the calibration
  Mat cam0( cameras[0].cam() ), cam1( cameras[1].cam() );
  Mat dist0( cameras[0].distCoeffs() ), dist1( cameras[1].distCoeffs() );

  cout << "cam0 before: " << endl << cam0 << endl;
  cout << "cam1 before: " << endl << cam1 << endl;

  cout << "dist0 before: " << endl << dist0 << endl;
  cout << "dist1 before: " << endl << dist1 << endl;

  int flags = CV_CALIB_FIX_INTRINSIC;
  //int flags = CALIB_USE_INTRINSIC_GUESS;

  reprojError = stereoCalibrate( objectPoints, imagePoints[0], imagePoints[1], 
      cam0, dist0, cam1, dist1,
      imageSize, r, t, e, f, 
      TermCriteria(TermCriteria::COUNT+TermCriteria::EPS, 30, 1e-6),
      flags );

cout << "cam0 after: " << endl << cam0 << endl;
  cout << "cam1 after: " << endl << cam1 << endl;

  cout << "dist0 after: " << endl << dist0 << endl;
  cout << "dist1 after: " << endl << dist1 << endl;



  cout << "R: " << endl << r << endl;
  cout << "T: " << endl << t << endl;
  cout << "E: " << endl << e << endl;
  cout << "F: " << endl << f << endl;


  Mat qx, qy, qz;
  Mat Rq, Qq;

  RQDecomp3x3( r, Rq, Qq, qx, qy, qz );
  cout << "qx: " << endl << qx << endl;
  cout << "qy: " << endl << qy << endl;
  cout << "qz: " << endl << qz << endl;

  cout << "Euler around x: " << acos( qx.at<double>(1,1) ) * 180.0/M_PI << endl;
  cout << "Euler around y: " << acos( qy.at<double>(0,0) ) * 180.0/M_PI << endl;
  cout << "Euler around z: " << acos( qz.at<double>(0,0) ) * 180.0/M_PI << endl;
  
  Mat r0, r1, p0, p1, disparity;

  // Hartley method...
  //
  // NEEDS undistorted points
  vector<Point2f> allimgpt[2];
  for( int k = 0; k < 2; ++k )
    for( int  i = 0; i < imagePoints[k].size(); i++ )
      std::copy(imagePoints[k][i].begin(), imagePoints[k][i].end(), back_inserter(allimgpt[k]));

  f = findFundamentalMat(Mat(allimgpt[0]), Mat(allimgpt[1]), FM_8POINT, 0, 0);
  Mat H1, H2;
  stereoRectifyUncalibrated(Mat(allimgpt[0]), Mat(allimgpt[1]), f, imageSize, H1, H2, 3);

cout << "Hartley F: " << endl << f << endl;

  r0 = cam0.inv()*H1*cam0;
  r1 = cam1.inv()*H2*cam1;
  p0 = cam0;
  p1 = cam1;


//  Rect validROI[2];
//  stereoRectify( cam0, dist0, cam1, dist1, imageSize, r, t,
//      r0, r1, p0, p1,  disparity, CALIB_ZERO_DISPARITY, -1, imageSize, &validROI[0], &validROI[1] );

  cout << "r0: " << endl << r0 << endl;
  cout << "p0: " << endl << p0 << endl;
  cout << "r1: " << endl << r1 << endl;
  cout << "p1: " << endl << p1 << endl;

  Mat map01, map02;
  Mat map11, map12;

  initUndistortRectifyMap(cam0, dist0, r0, p0,
      imageSize, CV_16SC2, map01, map02);

  initUndistortRectifyMap(cam1, dist1, r1, p1,
      imageSize, CV_16SC2, map11, map12);

  cout << "map01: " << endl << map01 << endl;
  cout << "map02: " << endl << map02 << endl;
  cout << "map11: " << endl << map11 << endl;
  cout << "map12: " << endl << map12 << endl;

  for( int i = 0; i < images.size(); ++i ) {

    Mat canvas( std::max( images[i].first.size().height, images[i].second.size().height ),
        images[i].first.size().width + images[i].second.size().width,
        images[i].first.img().type() );
    Mat rectified[2] = {
      Mat( canvas, Rect( Point(0,0), images[i].first.size()) ),
      Mat( canvas, Rect(Point( images[i].first.size().width, 0 ), images[i].second.size() )) };

    remap( images[i].first.img(), rectified[0], map01, map02, INTER_LINEAR );
    remap( images[i].second.img(), rectified[1], map11, map12, INTER_LINEAR );

    string outfile( opts.tmpPath( String("rectified/") +  images[i].first.basename() + ".jpg" ) );
    mkdir_p( outfile );
    imwrite(  outfile, canvas );

  }

  //      if( detection->points.size() > 0 ) {
  //        imagesUsed[i].push_back( img );
  //        imagePoints[i].push_back( detection->points );
  //        objectPoints[i].push_back( detection->corners );
  //
  //        detection->drawCorners(  *board, view );
  //      }
  //
  //      string outfile( opts.tmpPath( img.basename() ) );
  //      mkdir_p( outfile );
  //      imwrite(  outfile, view );
  //
  //      delete detection;
  //    }
  //

  //
  //
  //  cout << "Using points from " << imagePoints.size() << "/" << opts.inFiles.size() << " images" << endl;
  //
  //  string cameraFile( opts.cameraPath(mkCameraFileName() ) );
  //  vector<Mat> rvecs, tvecs;
  //  vector<float> reprojErrs;
  //  double totalAvgErr = 0;
  //
  //  bool ok = runCalibration(imagePoints, objectPoints,
  //      imageSize, aspectRatio, flags, cameraMatrix, distCoeffs,
  //      rvecs, tvecs, reprojErrs, totalAvgErr);
  //  printf("%s. avg reprojection error = %.2f\n",
  //      ok ? "Calibration succeeded" : "Calibration failed",
  //      totalAvgErr);
  //
  //  if( ok ) {
  //    cout << "Writing results to " << cameraFile << endl;
  //    saveCameraParams( cameraFile, imageSize,
  //        *board, imagesUsed, aspectRatio,
  //        flags, cameraMatrix, distCoeffs,
  //        writeExtrinsics ? rvecs : vector<Mat>(),
  //        writeExtrinsics ? tvecs : vector<Mat>(),
  //        writeExtrinsics ? reprojErrs : vector<float>(),
  //        writePoints ? imagePoints : vector<vector<Point2f> >(),
  //        totalAvgErr );
  //  }

delete board;

return 0;
}
