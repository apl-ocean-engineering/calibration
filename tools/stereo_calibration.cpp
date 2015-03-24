#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <iostream>
#include <algorithm>
#include <vector>

#include <boost/filesystem.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>


#include <cctype>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <getopt.h>


#include "file_utils.h"
#include "board.h"
#include "detection.h"
#include "image.h"
#include "camera_factory.h"
#include "stereo_pair.h"

using namespace cv;
using namespace std;

using namespace Distortion;

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

    const string stereoPath( void )
    { return dataDir + "/stereo"; }


    const string imageCache( const Image &image, const string &suffix="" )
    { return cachePath() + "/" + image.hash() + suffix + ".yml"; }

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

class StereoCalibration {
  public:
    StereoCalibration() {;}

    Mat e, f, R, t;
};

static void saveStereoCalibration( const string& filename,
const string &cameraFile0, const string &cameraFile1,
const StereoCalibration &cal )
//    Size imageSize, const Board &board,
//    const vector< Image > &imagesUsed,
//    float aspectRatio, int flags,
//    const Mat& cameraMatrix, const Mat& distCoeffs,
//    const vector<Mat>& rvecs, const vector<Mat>& tvecs,
//    const vector<float>& reprojErrs,
//    const vector<vector<Point2f> >& imagePoints,
//    double totalAvgErr )
{

  FileStorage out( filename, FileStorage::WRITE );

  time_t tt;
  time( &tt );
  struct tm *t2 = localtime( &tt );
  char buf[1024];
  strftime( buf, sizeof(buf)-1, "%c", t2 );

  out << "calibration_time" << buf;

  out << "camera_0" << cameraFile0;
  out << "camera_1" << cameraFile1;

  out << "fundamental" << cal.f;
  out << "essential" << cal.e;

  out << "rotation" << cal.R;
  out << "translation" << cal.t;

cout << "Wrote stereo pair calibration to " << filename << endl;


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



static string mkStereoPairFileName( const string &cam0, const string &cam1 )
{
  char strtime[32], filename[80];
  time_t tt;
  time( &tt );
  struct tm *t2 = localtime( &tt );
  strftime( strtime, 32, "%y%m%d_%H%M%S", t2 );
  snprintf( filename, 80, "cal_%s_%s_%s.yml", cam0.c_str(), cam1.c_str(), strtime );
  return  string( filename );
}


class ImagePair
{
  public:
    ImagePair( const Image &a, const Image &b )
      : _a(a), _b(b) {;}

    const Image &operator[](int i) {
      switch(i) {
        case 0: return _a; break;
        case 1: return _b; break;
      }
    }

  private:
    Image _a, _b;
};

struct CompositeCanvas
{
  CompositeCanvas( const ImagePair &pair )
    : _pair( pair ) 
  {
    // I really should do this with undistorted images...
    canvas.create( std::max( _pair[0].size().height, _pair[1].size().height ),
        _pair[0].size().width + _pair[1].size().width,
        _pair[0].img().type() );

    origin[0] = Point2f(0,0);
    origin[1] = Point2f( _pair[0].size().width, 0 );
    roi[0] = Mat( canvas, Rect( origin[0], _pair[0].size()) );
    roi[1] = Mat( canvas, Rect( origin[1], _pair[1].size()) );
  }

  operator Mat &() { return canvas; }
  operator _InputArray() { return _InputArray(canvas); }

  Size size( void ) const { return canvas.size(); }


  ImagePair _pair;
  Mat canvas, roi[2];
  Point2f origin[2];
};


struct TxRectifier {
  TxRectifier( const Mat &r )
    : _r( r) {;}
  const Mat &_r;

  ImagePoint operator()( const ImagePoint &pt )
  {
    Vec3d p( pt[0], pt[1], 1.0 );
    Mat r = _r * Mat(p);
    return ImagePoint( r.at<double>(0,0)/r.at<double>(2,0), r.at<double>(1,0)/r.at<double>(2,0) );
  }
};


int main( int argc, char** argv )
{

  StereoCalibrationOpts opts( argc, argv );

  Board *board = Board::load( opts.boardPath(), opts.boardName );

  float aspectRatio = 1.f;
  bool writeExtrinsics = false, writePoints = false;

  int numPoints = 0;
  ImagePointsVecVec imagePoints[2];
  ImagePointsVecVec undistortedImagePoints[2];

  // Make ``flattened'' versions as well
  ImagePointsVec undistortedImagePts[2];

  ObjectPointsVecVec objectPoints;

  if( opts.inFiles.size() < 1 ) {
    cout << "No input files specified on command line." << endl;
    exit(-1);
  }

  string cameraCalibrationFiles[2] = { opts.cameraLatest(0), opts.cameraLatest(1) };

  cout << "Loading camera 1 from: " << cameraCalibrationFiles[0] << endl;
  cout << "Loading camera 2 from: " << cameraCalibrationFiles[1] << endl;

  // Load the camera files
  DistortionModel *cameras[2] = { CameraFactory::LoadDistortionModel( cameraCalibrationFiles[0] ),
    CameraFactory::LoadDistortionModel( cameraCalibrationFiles[1] ) };

  for( int i = 0; i < 2; ++i ) 
    if( !cameras[i] ) {
      cerr << "Couldn't load calibration for camera \"" << opts.cameraName[i] << endl;
      exit(-1);
    }

  cout << "Camera 1 is a " << cameras[0]->name() << endl;
  cout << "Camera 2 is a " << cameras[1]->name() << endl;

  vector< ImagePair > pairsUsed;
  vector< ImagePair > pairs;

  if( opts.ignoreCache ) cout << "Ignoring cached data." << endl;

  for( int i = 0; i < opts.inFiles.size(); ++i ) {
    cout << "Processing " << i << " : " << opts.inFiles[i] << endl;
    Mat view, viewGray;

    view = imread(opts.inFiles[i], 1);


    // Break out two ROIs
    int roiWidth = view.size().width/2;
    Size roiSize( roiWidth, view.size().height );

    Mat rois[2] = { Mat( view, Rect( Point(0, 0), roiSize ) ),
      Mat( view, Rect( Point(roiWidth, 0), roiSize ) ) };

    Image compositeImage( opts.inFiles[i], view );

    ImagePair pair( Image( opts.inFiles[i] + "-left", rois[0] ),
        Image( opts.inFiles[i] + "-right", rois[1] ) );
    pairs.push_back( pair );

    // Technically speaking you could eager load the images ... you don't even need them
    //  if you're reading from the cache.

    // Check for cached data
    bool doRegister = true;
    Detection *detection[2] = { NULL, NULL };

    for( int i = 0; i < 2; ++i ) {

      string suffix = (i==0 ? "-left" : "-right");
      if( !opts.ignoreCache && (detection[i] = Detection::loadCache(  opts.imageCache( compositeImage, suffix ) )) != NULL ) {
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

        detection[i]->writeCache( *board, opts.imageCache( compositeImage, suffix ) );
      }
    }

    if( detection[0] != NULL || detection[1] != NULL ) {
      // Find the overlapping tags between each

      SharedPoints shared( Detection::sharedWith( *detection[0], *detection[1] ) );

      assert( (shared.worldPoints.size() != shared.imagePoints[0].size()) ||
          (shared.worldPoints.size() == shared.imagePoints[1].size() ) );

      if( shared.worldPoints.size() > 0 ) {
        objectPoints.push_back( shared.worldPoints );
        numPoints += shared.worldPoints.size();

        //:ImagePointsVec undistortedPoints[2] = {
        //:  ImagePointsVec( shared.imagePoints[0].size() ), 
        //:  ImagePointsVec( shared.imagePoints[1].size() )  };

        // Generate undistorted image points as well
        for( int imgIdx = 0; imgIdx < 2 ; ++imgIdx ) {
          imagePoints[imgIdx].push_back( shared.imagePoints[imgIdx] );

          // Technically this is normalize->undistort->reimage
          ImagePointsVec undist = cameras[imgIdx]->undistort( shared.imagePoints[imgIdx] );

          undistortedImagePoints[imgIdx].push_back( undist );
          std::copy( undist.begin(), undist.end(), back_inserter( undistortedImagePts[imgIdx] ) );
        }

        ImagePair &thisPair( pairs[i] );
        CompositeCanvas canvas( thisPair );

        for( int imgIdx = 0; imgIdx < 2 ; ++imgIdx ) {
          cameras[imgIdx]->undistortImage( thisPair[imgIdx].img(), canvas.roi[imgIdx] );
        }

        //thisPair[0].img().copyTo( rectified[0] );
        //images[i].second.img().copyTo( rectified[1] );


        for( int j = 0; j < shared.worldPoints.size(); ++j ) {
          //cout << shared.worldPoints[j] << undistortedPoints[0][j] << undistortedPoints[1][j] << endl;
          //cout << shared.worldPoints[j] << shared.aPoints[j] << shared.bPoints[j] << endl;
          float i = (float)j / shared.worldPoints.size();
          Scalar color( 255*i, 0, (1-i)*255);

          cv::circle( canvas.roi[0], Point(undistortedImagePoints[0].back()[j]), 5, color, -1 );
          cv::circle( canvas.roi[1], Point(undistortedImagePoints[1].back()[j]), 5, color, -1 );

          cv::line(  canvas, Point(undistortedImagePoints[0].back()[j]), 
              canvas.origin[1] + Point2f(undistortedImagePoints[1].back()[j]),
              color, 1 );
        }

        string outfile( opts.tmpPath( String("annotated/") +  pairs[i][0].basename() + ".jpg" ) );
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
  Mat R[2], P[2], disparity;
  Rect validROI[2];

  Size imageSize( pairs[0][0].size( ) );
  //cout << "image size: " <<  imageSize << endl;

  // Local copies as they might be changed in the calibration
  Mat cam[2] = { cameras[0]->mat(), cameras[1]->mat() };
  //Mat dist[2] = { cameras[0].distCoeffs(), cameras[1].distCoeffs() };

  cout << "cam0 before: " << endl << cam[0] << endl;
  cout << "cam1 before: " << endl << cam[1] << endl;

  //cout << "dist0 before: " << endl << dist[0] << endl;
  //cout << "dist1 before: " << endl << dist[1] << endl;

  enum { STEREO_CALIBRATE, HARTLEY } method = HARTLEY;

  if( method == HARTLEY ) {
    cout << "!!! Using Hartley !!!" << endl;

    // Hartley method calculates F directly.  Then decomposes that into E and decomposes
    // that into T and R
    //
    // In this case, assume the cameras are calibrated properly, find E directly, then
    // generate F, T, R
    //
    // NEEDS a single vector of undistorted points
    // Make a set of flattened, undistorted points
    //  and a set of flattened, undistorted, normalized points
    ImagePointsVec normimgpt[2];

    for( int k = 0; k < 2; ++k )
      std::transform(undistortedImagePts[k].begin(), undistortedImagePts[k].end(), 
          back_inserter(normimgpt[k]), cameras[k]->makeNormalizer()  );


    Mat status, estF;
    estF = findFundamentalMat(Mat(undistortedImagePts[0]), Mat(undistortedImagePts[1]), FM_RANSAC, 3., 0.99, status);
    cout << "Est F: " << endl << estF << endl;


    Mat estE;
    estE = findFundamentalMat(Mat(normimgpt[0]), Mat(normimgpt[1]), FM_RANSAC, 1./1600, 0.99, status);

    cout << "estimated e: " << endl << estE << endl;
    // Normalize e
    SVD svdE( estE );
    Matx33d idealSigma( 1,0,0,
        0,1,0,
        0,0,0 );
    cout << "eigenvalues of calculated e: " << svdE.w << endl;
    e = svdE.u * Mat(idealSigma) * svdE.vt;
    e /= (e.at<double>(2,2) == 0 ? 1.0 : e.at<double>(2,2) );
    cout << "ideal e: " << e << endl;


    int count = 0;
    for( int i = 0; i < status.size().area(); ++i ) 
      if( status.at<unsigned int>(i,0) > 0 ) ++count;
    cout << count << "/" << undistortedImagePts[0].size() << " points considered inlier." << endl;

    // Calculate the mean reprojection error under this f
    double error = 0;
    for( int i = 0; i < normimgpt[0].size(); ++i ) {
      if( status.at<uint8_t>(i) > 0 ) {
        Mat err( Mat(Vec3d( normimgpt[1][i][0], normimgpt[1][i][1], 1.0 )).t() *
            e * Mat(Vec3d( normimgpt[0][i][0], normimgpt[0][i][1], 1.0 ) ) );
        error += pow(err.at<double>(0,0),2);
      }
    }

    error /= count;
    cout << "Mean E reproj error " << error << endl;

    // Calculate f
    f = cameras[1]->mat().inv().t() * e * cameras[0]->mat().inv();
    f /= (f.at<double>(2,2) == 0 ? 1.0 : f.at<double>(2,2) );

    // Calculate the mean reprojection error under this f
    error = 0.0;
    for( int i = 0; i < undistortedImagePts[0].size(); ++i ) {
      if( status.data[i] > 0 ) {
        Mat err(  Mat(Vec3d( undistortedImagePts[1][i][0], undistortedImagePts[1][i][1], 1.0 ) ) .t() *
            f * Mat(Vec3d( undistortedImagePts[0][i][0], undistortedImagePts[0][i][1], 1.0 ) ) );
        error += pow(err.at<double>(0,0),2);
      }
    }

    error /= count;
    cout << "Mean F reproj error " << error << endl;

    // Disambiguate the four solutions by:
    // http://stackoverflow.com/questions/22807039/decomposition-of-essential-matrix-validation-of-the-four-possible-solutions-for
    Mat W( Matx33d(0,-1,0,
          1,0,0,
          0,0,1) );
    Mat u( svdE.u ), vt( svdE.vt );
    if( determinant(u) < 0 )  u*= -1;
    if( determinant(vt) < 0 ) vt *= -1;
    Mat rcand = u * W * vt;
    Mat tcand = u.col(2);

    bool done = false;
    while( !done ) {
      Mat M = rcand.t() * tcand;
      double m1 = M.at<double>(0), m2 = M.at<double>(1), m3 = M.at<double>(2);
      Mat Mx = (Mat_<double>(3,3) << 0, -m3, m2, m3, 0, -m1, -m2, m1, 0 );

      Point3d x1( normimgpt[0][0][0], normimgpt[0][0][1], 1.0 ),
              x2( normimgpt[1][0][0], normimgpt[1][0][1], 1.0 );

      Mat X1 = Mx * Mat(x1),
          X2 = Mx * rcand.t() * Mat(x2);

      if( (X1.at<double>(2) * X2.at<double>(2)) < 0 ) 
        rcand = u * W.t() * vt;
      else if (X1.at<double>(2) < 0) 
        tcand *= -1;
      else 
        done = true;
    }


    r = rcand;
    t = tcand;

    //Mat H[2];
    //stereoRectifyUncalibrated(Mat(allimgpt[0]), Mat(allimgpt[1]), e, imageSize, H[0], H[1], 3);

    //for( int k = 0; k < 2; ++k ) {
    //  R[k] = cam[k].inv()*H[k]*cam[k];
    //  P[k] = cam[k];
    //}

  } else if( method == STEREO_CALIBRATE ) {
    cout << "!!! Using stereoCalibrate !!!" << endl;

    int flags = CV_CALIB_FIX_INTRINSIC;

    // For what it's worth, cvStereoCalibrate appears to optimize for the translation and rotation
    // (and optionally the intrinsics) by minimizing the L2-norm reprojection error
    // Then computes E directly (as [T]_x R) then F = K^-T E F^-1
    reprojError = Distortion::stereoCalibrate( objectPoints, imagePoints[0], imagePoints[1], 
        *cameras[0], *cameras[1],
        imageSize, r, t, e, f, 
        TermCriteria(TermCriteria::COUNT+TermCriteria::EPS, 30, 1e-6),
        flags );

    SVD svd(e);
    cout << "sigma: " << svd.w << endl;
    //
    //cout << "cam0 after: " << endl << cam0 << endl;
    //  cout << "cam1 after: " << endl << cam1 << endl;
    //
    //  cout << "dist0 after: " << endl << dist0 << endl;
    //  cout << "dist1 after: " << endl << dist1 << endl;

    cout << "Reprojection error: " << reprojError << endl;
  } else {
    cout << "No method specified.  Stopping..." << endl;
    exit(0);
  }


  StereoCalibration cal;
  cal.f = f;
  cal.e = e;
  cal.R = r;
  cal.t = t;

  saveStereoCalibration( opts.stereoPath() + mkStereoPairFileName( opts.cameraName[0], opts.cameraName[1] ),
      cameraCalibrationFiles[0], cameraCalibrationFiles[1],
      cal );

  float alpha = -1;
  Distortion::stereoRectify( *cameras[0], *cameras[1], imageSize, r, t,
      R[0], R[1], P[0], P[1],  disparity, CALIB_ZERO_DISPARITY, 
      alpha, imageSize, validROI[0], validROI[1] );

  cout << "R: " << endl << r << endl;
  cout << "T: " << endl << t << endl;
  cout << "norm(T): " << norm(t, NORM_L2 ) << endl;
  cout << "E: " << endl << e << endl;
  cout << "F: " << endl << f << endl;


  Mat H[2];
  stereoRectifyUncalibrated( undistortedImagePts[0], undistortedImagePts[1], f, imageSize, H[0], H[1] );
  cout << "H[0]: " << endl << H[0] << endl;
  cout << "H[1]: " << endl << H[1] << endl;

  //
  //
  //  Mat qx, qy, qz, Rq, Qq;
  //
  //  RQDecomp3x3( r, Rq, Qq, qx, qy, qz );
  //  //  cout << "qx: " << endl << qx << endl;
  //  //  cout << "qy: " << endl << qy << endl;
  //  //  cout << "qz: " << endl << qz << endl;
  //
  //  cout << "Euler around x: " << acos( qx.at<double>(1,1) ) * 180.0/M_PI << endl;
  //  cout << "Euler around y: " << acos( qy.at<double>(0,0) ) * 180.0/M_PI << endl;
  //  cout << "Euler around z: " << acos( qz.at<double>(0,0) ) * 180.0/M_PI << endl;
  //
  cout << "r0: " << endl << R[0] << endl;
  cout << "p0: " << endl << P[0] << endl;
  cout << "r1: " << endl << R[1] << endl;
  cout << "p1: " << endl << P[1] << endl;

  //  cout << "r0^T r1: " << endl << R[0].t() * R[1] << endl;
  //  cout << "r0 T: " << endl << R[0] * t << endl;
  //  cout << "r0^T T: " << endl << R[0].t() * t << endl;
  //  cout << "r1 T: " << endl << R[1] * t << endl;
  //  cout << "r1^T T: " << endl << R[1].t() * t << endl;

  // Generate undistorted images


  Mat map[2][2];
  for( int k = 0; k < 2; ++k ) { 
    cameras[k]->initUndistortRectifyMap( H[k], cameras[k]->mat(), //P[k],
        imageSize, CV_32FC1, map[k][0], map[k][1] );
    //cameras[k]->initUndistortRectifyMap( R[k], P[k],
    //    imageSize, CV_32FC1, map[k][0], map[k][1] );

    //cout << "map" << k << "0: " << endl << map[k][0] << endl;
    //cout << "map" << k << "1: " << endl << map[k][1] << endl;
  }

  for( int i = 0; i < pairs.size(); ++i ) {

    ImagePair &thisPair( pairs[i] );
    CompositeCanvas canvas( thisPair );

    for( int idx = 0; idx < 2; ++idx ) {
      Mat undist;
      cameras[idx]->undistortImage( thisPair[idx].img(), undist );
      warpPerspective( undist, canvas.roi[idx], H[idx], imageSize );

     // remap( thisPair[idx].img(), canvas.roi[idx], map[idx][0], map[idx][1], INTER_LINEAR );

     // Scalar roiBorder( 0,255,0 );
     // line( canvas.roi[idx], Point(validROI[idx].x, validROI[idx].y), Point(validROI[idx].x+validROI[idx].width, validROI[idx].y), roiBorder, 1 ); 
     // line( canvas.roi[idx], Point(validROI[idx].x+validROI[idx].width, validROI[idx].y), Point(validROI[idx].x+validROI[idx].width, validROI[idx].y+validROI[idx].height), roiBorder, 1 ); 
     // line( canvas.roi[idx], Point(validROI[idx].x+validROI[idx].width, validROI[idx].y+validROI[idx].height), Point(validROI[idx].x, validROI[idx].y+validROI[idx].height), roiBorder, 1 ); 
     // line( canvas.roi[idx], Point(validROI[idx].x, validROI[idx].y+validROI[idx].height), Point(validROI[idx].x, validROI[idx].y), roiBorder, 1 ); 

    }


    // Draw the standard red horizontal lines
    int spacing = 200;
    for( int y = 0; y < canvas.size().height; y += spacing ) 
      line( canvas, Point( 0, y ), Point( canvas.size().width, y ), Scalar( 0,0,255 ), 2 );


    string outfile( opts.tmpPath( String("stereo_rectified/") +  thisPair[0].basename() + ".jpg" ) );
    mkdir_p( outfile );
    imwrite(  outfile, canvas );

  }

  //  {
  //    pcl::PointCloud<pcl::PointXYZRGB> cloud;
  //
  //    cloud.width = numPoints;
  //    cloud.height = 1;
  //    cloud.points.resize( cloud.width * cloud.height );
  //
  //    int at = 0;
  //
  //    //Think about some reconstruction
  //    for( int i = 0; i < pairs.size(); ++i ) {
  //      Mat worldPoints;
  //
  //    vector< ImagePoint > rectifiedPoints[2];
  //
  //    for( int j = 0; j < 2; ++j ) {
  //      std::transform( undistortedImagePoints[j][i].begin(), undistortedImagePoints[j][i].end(),
  //          back_inserter( rectifiedPoints[j] ), cameras[j]->makeNormalizer() );
  //
  //      Mat cam( P[0], Rect(0,0,3,3) );
  //      std::transform( rectifiedPoints[j].begin(), rectifiedPoints[j].end(),
  //          rectifiedPoints[j].begin(), TxRectifier( cam*R[j] ) );
  //    }
  //
  //
  //      triangulatePoints( P[0], P[1],rectifiedPoints[0], rectifiedPoints[1], worldPoints );
  //
  //      //cout << "World points: " << worldPoints << endl;
  //
  //      for( int j = 0; j < worldPoints.cols; ++j ) {
  //        Vec4f pt;
  //        worldPoints.col(j).convertTo( pt, CV_32F );
  //
  //        cloud.points[at].x = pt[0]/pt[3];
  //        cloud.points[at].y = pt[1]/pt[3];
  //        cloud.points[at].z = pt[2]/pt[3];
  //
  //        uint8_t r = 255 * ((float)i / (float)pairs.size() ),
  //                g = 255-r, b = 255-r;
  //
  //        uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
  //        cloud.points[at].rgb = *reinterpret_cast<float*>(&rgb);
  //
  //        ++at;
  //      }
  //    }
  //
  //    pcl::io::savePCDFileASCII ("test_pcd.pcd", cloud);
  //    std::cerr << "Saved " << cloud.points.size () << " data points to test_pcd.pcd." << std::endl;
  //  }

  {
    pcl::PointCloud<pcl::PointXYZRGB> cloud;

    cloud.width = numPoints;
    cloud.height = 1;
    cloud.points.resize( cloud.width * cloud.height );

    int at = 0;

    Mat newP0 = Mat::eye( 3,4, CV_64F ), newP1( 3,4,CV_64F );
    Mat subR( newP1, Rect(0,0,3,3) ), subT( newP1.col(3) );

    r.copyTo( subR );
    t.copyTo( subT );

    //Think about some reconstruction
    for( int i = 0; i < pairs.size(); ++i ) {
      Mat worldPoints;

      triangulatePoints( cameras[0]->mat()*newP0, cameras[1]->mat()*newP1, 
          undistortedImagePoints[0][i], undistortedImagePoints[1][i], worldPoints );

      //cout << "World points: " << worldPoints << endl;

      for( int j = 0; j < worldPoints.cols; ++j ) {
        Vec4f pt;
        worldPoints.col(j).convertTo( pt, CV_32F );

        cloud.points[at].x = pt[0]/pt[3];
        cloud.points[at].y = pt[1]/pt[3];
        cloud.points[at].z = pt[2]/pt[3];

        uint8_t r = 255 * ((float)i / (float)pairs.size() ),
                g = 255-r, b = 255-r;

        uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
        cloud.points[at].rgb = *reinterpret_cast<float*>(&rgb);

        ++at;
      }
    }

    pcl::io::savePCDFileASCII ("test_pcd_rectified.pcd", cloud);
    std::cerr << "Saved " << cloud.points.size () << " data points to test_pcd_rectified.pcd." << std::endl;
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
if( cameras[0] ) delete cameras[0];
if( cameras[1] ) delete cameras[1];

return 0;
}
