#include <iostream>
#include <algorithm>
#include <vector>

#include <opencv2/calib3d.hpp>

#include "file_utils.h"

#include <tclap/CmdLine.h>
#include <glog/logging.h>

#include "board.h"
#include "detection_db.h"
using namespace AplCam;

#include "distortion/distortion_model.h"
#include "distortion/camera_factory.h"
#include "distortion/distortion_stereo.h"
using namespace Distortion;

#include "stereo_calibration.h"

using namespace cv;
using namespace std;




struct DbStereoCalibrationOpts {
 public:
  DbStereoCalibrationOpts()
  {;}

  string boardName, dataDir, detectionDb[2], cameraCalibrations[2], stereoCalOutput;

  const string boardPath( void )
  { return dataDir + "/boards/" + boardName + ".yml"; }

  const string tmpPath( const string &file )
  { return dataDir + "/tmp/" + file; }

  const string cachePath( const string &file = "" ) const
  { return dataDir + "/cache/" + file; }

  const string cameraCalibrationPath( const int i )
  { return cameraCalibrations[i]; }

  bool parseOpts( int argc, char **argv )
  {

    try {

      TCLAP::CmdLine cmd("db_stereo_calibration", ' ', "0.1");

      TCLAP::ValueArg<std::string> dataDirArg( "d", "data-dir", "Data directory", false, "data/", "dir", cmd );
      TCLAP::ValueArg<std::string> detectionDbLeftArg( "", "detection-db-left", "Detection db directory", false, ".", "dir", cmd );
      TCLAP::ValueArg<std::string> detectionDbRightArg( "", "detection-db-right", "Detection db directory", false, ".", "dir", cmd );
      TCLAP::ValueArg<std::string> boardNameArg( "b", "board-name", "Board name", true, "", "dir", cmd );
      TCLAP::ValueArg<std::string> calLeftArg("0","camera-left", "Calibration file basename", true, "", "name", cmd );
      TCLAP::ValueArg<std::string> calRightArg("1","camera-right", "Calibration file basename", true, "", "name", cmd );
      TCLAP::ValueArg<std::string> stereoCalArg( "o", "calibration-output", "Filename for resulting stereo calibration", true, "", "dir", cmd );

      cmd.parse( argc, argv );

      dataDir = dataDirArg.getValue();
      detectionDb[0] = detectionDbLeftArg.getValue();
      detectionDb[1] = detectionDbRightArg.getValue();
      boardName = boardNameArg.getValue();
      cameraCalibrations[0] = calLeftArg.getValue();
      cameraCalibrations[1] = calRightArg.getValue();
      stereoCalOutput = stereoCalArg.getValue();

    } catch( TCLAP::ArgException &e ) {
      LOG( ERROR ) << "error: " << e.error() << " for arg " << e.argId();
    }

    return validate();
  }

  bool validate( void )
  {
    if( !file_exists( boardPath() ) ) {
      LOG(ERROR) << "Can't find board file: " << boardPath();
      return false;
    }

    for( int i = 0; i < 2; ++i ) {
      if( !file_exists( detectionDb[i] ) ) {
        LOG(ERROR )<< "Detection db \"" << detectionDb[i] << "\" doesn't exist.";
        return false;
      }
    }


    return true;
  }

};

// struct RectifierFunctor {
//   RectifierFunctor( const Mat &r )
//       : _r( r) {;}
//   const Mat &_r;
//
//   ImagePoint operator()( const ImagePoint &pt )
//   {
//     Vec3d p( pt[0], pt[1], 1.0 );
//     Mat r = _r * Mat(p);
//     return ImagePoint( r.at<double>(0,0)/r.at<double>(2,0), r.at<double>(1,0)/r.at<double>(2,0) );
//   }
// };


struct FlatCalibrationData {
  FlatCalibrationData() {;}

  size_t size( void ) const { return objectPoints_.size(); }

  void clear( void )
  {
    objectPoints_.clear();
    imagePoints_[0].clear();
    imagePoints_[1].clear();
  }

  void normalize( FlatCalibrationData &dest, DistortionModel &cama, DistortionModel &camb )
  {
    dest.clear();
    dest.objectPoints_ = objectPoints_;
    std::transform(imagePoints_[0].begin(), imagePoints_[0].end(),
                   back_inserter(dest.imagePoints_[0]), cama.makeNormalizer() );

    std::transform(imagePoints_[1].begin(), imagePoints_[1].end(),
                   back_inserter(dest.imagePoints_[1]), camb.makeNormalizer() );
  }

  void image( FlatCalibrationData &dest, DistortionModel &cama, DistortionModel &camb )
  {
    dest.clear();
    dest.objectPoints_ = objectPoints_;
    std::transform(imagePoints_[0].begin(), imagePoints_[0].end(),
                   back_inserter(dest.imagePoints_[0]), cama.makeImager() );

    std::transform(imagePoints_[1].begin(), imagePoints_[1].end(),
                   back_inserter(dest.imagePoints_[1]), camb.makeImager() );
  }


  void keepIf( FlatCalibrationData &dest, vector<bool> &keep )
  {
    dest.clear();
    for( size_t i = 0; i < objectPoints_.size(); ++i  ) {
      if( keep[i] ) {
        dest.objectPoints_.push_back( objectPoints_[i] );
        dest.imagePoints_[0].push_back( imagePoints_[0][i] );
        dest.imagePoints_[1].push_back( imagePoints_[1][i] );
      }
    }

  }


  ImagePointsVec imagePoints_[2];
  ObjectPointsVec objectPoints_;
};


class StereoCalibrationData {
 public:

  StereoCalibrationData( void )
      : numPts_(0)
  {;}

  void clear( void )
  {
    numPts_ = 0;
    objectPoints_.clear();
    imagePoints_[0].clear();
    imagePoints_[1].clear();
  }

  void add( ObjectPointsVec &obj, ImagePointsVec &imga, ImagePointsVec &imgb )
  {
    objectPoints_.push_back( obj );
    imagePoints_[0].push_back( imga );
    imagePoints_[1].push_back( imgb );
    numPts_ += obj.size();
  }

  void undistort( StereoCalibrationData &dest, DistortionModel &cama, DistortionModel &camb )
  {
    dest.clear();

    for( size_t i = 0; i < objectPoints_.size(); ++i ) {
      ImagePointsVec undist[2] = {
        cama.normalizeUndistort( imagePoints_[0][i] ),
        camb.normalizeUndistort( imagePoints_[1][i] ) };

      dest.add( objectPoints_[i], undist[0], undist[1] );
    }
  }

  void image( StereoCalibrationData &dest, DistortionModel &cama, DistortionModel &camb )
  {
    dest.clear();

    for( size_t i = 0; i < objectPoints_.size(); ++i ) {
      ImagePointsVec undist[2] = {
        cama.image( imagePoints_[0][i] ),
        camb.image( imagePoints_[1][i] ) };

      dest.add( objectPoints_[i], undist[0], undist[1] );
    }
  }

  void flatten( FlatCalibrationData &dest )
  {
    for( size_t i = 0; i < objectPoints_.size(); ++i ) {

      std::copy( objectPoints_[i].begin(), objectPoints_[i].end(), back_inserter(  dest.objectPoints_ ) );
      std::copy( imagePoints_[0][i].begin(), imagePoints_[0][i].end(), back_inserter(  dest.imagePoints_[0] ) );
      std::copy( imagePoints_[1][i].begin(), imagePoints_[1][i].end(), back_inserter(  dest.imagePoints_[1] ) );
    }
  }


  int size( void ) const { return numPts_; }

  void addPoints( const Detection &a, const Detection &b )
  {
    SharedPoints shared( Detection::sharedWith( a, b ) );

    assert( (shared.worldPoints.size() != shared.imagePoints[0].size()) ||
           (shared.worldPoints.size() == shared.imagePoints[1].size() ) );

    if( shared.worldPoints.size() > 0 ) {
      objectPoints_.push_back( shared.worldPoints );
      numPts_ += shared.worldPoints.size();

      for( int imgIdx = 0; imgIdx < 2 ; ++imgIdx ) {
        imagePoints_[imgIdx].push_back( shared.imagePoints[imgIdx] );

      }
    }
  }

  int numPts_;
  ImagePointsVecVec imagePoints_[2];
  ObjectPointsVecVec objectPoints_;

  //    ImagePointsVecVec undistortedImagePoints_[2];
  ////    // Make ``flattened'' versions as well
  //    ImagePointsVec undistortedImagePts_[2];

};



class DbStereoCalibration {
 public:
  DbStereoCalibration( DbStereoCalibrationOpts &opts )
      : opts_( opts ) {;}

  int run( void )
  {
    Board *board = Board::load( opts_.boardPath(), opts_.boardName );

    for( int i = 0; i < 2; ++i ) {
      cameras_[i] = CameraFactory::LoadDistortionModel( opts_.cameraCalibrationPath(i) );
      if( cameras_[i] == NULL ) {
        LOG(ERROR) << "Couldn't load camera from " << opts_.cameraCalibrationPath(i);
        return -1;
      }

      bool dbOpened;
      dbOpened = db_[i].open( opts_.detectionDb[i], false );

      if( !dbOpened ) {
        LOG(ERROR) << "Error opening database file " << opts_.detectionDb[i] << ": " << db_[i].error().name() << endl;
        return -1;
      }
    }

    doCalibrate();

    return 0;
  }


  void doCalibrate( void )
  {

    StereoCalibrationData calData, unData;

    // Use all data for now
    int vidLength = min( db_[0].vidLength(), db_[1].vidLength() );

    for( int frame = 0; frame < vidLength; ++frame ) {
if( frame % 100 == 0 ) LOG(INFO) << "Loaded frame: " << frame;
      Detection *det[2] = { db_[0].load( frame ), db_[1].load( frame ) };

      if( det[0] != NULL || det[1] != NULL ) calData.addPoints( *det[0], *det[1] );

      if( det[0] ) delete det[0];
      if( det[1] ) delete det[1];
    }

    LOG(INFO) << "Have created set of " << calData.size() << " points.";

    // unData is Unidistorted and Normalized
    calData.undistort( unData, *cameras_[0], *cameras_[1] );

    FlatCalibrationData flatUNData;
    unData.flatten( flatUNData );

    StereoCalibration cal;

    if( true ) {
      hartleyMethod( flatUNData, cal );
    } else {
      LOG(ERROR) << "Hmm, OpenCV method mot working at present.";
      //opencvMethod( calData, opencvCal );
    }


    // Solve scale
    ObjectPointsVecVec triangPts;
    vector< double > scales;
    for( size_t i = 0; i < calData.objectPoints_.size(); ++i ) {

      ImagePointsVec &imgPts0 = calData.imagePoints_[0][i],
                     &imgPts1 = calData.imagePoints_[1][i];
      ObjectPointsVec &objPts   = calData.objectPoints_[i];

      // Only estimate scale if two or more detections...
      if( objPts.size() < 2 ) continue;

      ObjectPointsVec tri;
      if( Distortion::triangulate( *cameras_[0], *cameras_[1], cal, imgPts0, imgPts1, tri ) == false ) continue;

      triangPts.push_back( tri );

      for( size_t j = 1; j < objPts.size(); ++j ) {
        // Estimate scale
        ObjectPoint dDet = objPts[j] - objPts[0];
        double delDet = dDet.ddot( dDet );

        ObjectPoint dTri = tri[j] - tri[0];
        double delTri = dTri.ddot( dTri);

        double scale = sqrt(delDet / delTri);
        scales.push_back( scale );

        // LOG(INFO) << "dDet: " << dDet << "   delDet: " << delDet;
        // LOG(INFO) << "dTri: " << dTri << "   delTri: " << delTri;
        // LOG(INFO) << "Scale: " << scale;
      }

    }


    // Compute mean scale
    double meanScale = 0.0, varScale = 0.0;
    size_t sz = scales.size();
    for( size_t i = 0; i < sz; ++i ) { meanScale += scales[i]; }
    meanScale /= sz;
    for( size_t i = 0; i < sz; ++i ) {
      float f = scales[i] - meanScale;
      varScale += f*f;
    }
    varScale /= sz;


    LOG(INFO) << "Scale: " << meanScale << "   sigma: " << sqrt(varScale);

    cal.t *= meanScale;
    cal.dumpDecomp();

    // == Attempt to rectify images

    Mat R[2], P[2], disparity;
    Rect validROI[2];
    float alpha = -1;

    // TODO:  Fix hardcoded value
    const Size ImageSize( db_[0].imageSize() );
    Distortion::stereoRectify( *cameras_[0], *cameras_[1], ImageSize, cal.R, cal.t,
        R[0], R[1], P[0], P[1],  disparity, CALIB_ZERO_DISPARITY,
        alpha, ImageSize, validROI[0], validROI[1] );

    StereoRectification rect;
    rect.R[0] = R[0];
    rect.R[1] = R[1];
    rect.P[0] = P[0];
    rect.P[1] = P[1];
    rect.Q = disparity;

    LOG(INFO) << "Saving to " <<  opts_.stereoCalOutput;
    saveStereoCalibration( opts_.stereoCalOutput, cal, rect,
                           opts_.cameraCalibrationPath(0),
                           opts_.cameraCalibrationPath(1) );
  }

  void saveStereoCalibration( const string &filename,
      const StereoCalibration &cal,
      const StereoRectification &rect,
      const string &cam0, const string &cam1 )
  {

    FileStorage fs( filename, FileStorage::WRITE );

    time_t tt;
    time( &tt );
    struct tm *t2 = localtime( &tt );
    char buf[1024];
    strftime( buf, sizeof(buf)-1, "%c", t2 );

    fs << "calibration_time" << buf;
    fs << "calibration_0" << cam0;
    fs << "calibration_1" << cam1;

    cal.save( fs );
    rect.save( fs );

    cout << "Wrote stereo calibration to " << filename << endl;
  }



  bool opencvMethod( StereoCalibrationData &data,
                    StereoCalibration &cal )
  {
    cout << "!!! Using stereoCalibrate !!!" << endl;

    Mat r, t, e, f;
    int flags = CV_CALIB_FIX_INTRINSIC;
    Size imageSize( db_[0].imageSize() );

StereoCalibrationData imagedData;
data.image( imagedData, *cameras_[0], *cameras_[1] );

    // For what it's worth, cvDbStereoCalibrate appears to optimize for the translation and rotation
    // (and optionally the intrinsics) by minimizing the L2-norm reprojection error
    // Then computes E directly (as [T]_x R) then F = K^-T E F^-1
    double reprojError = Distortion::stereoCalibrate( imagedData.objectPoints_, imagedData.imagePoints_[0], imagedData.imagePoints_[1],
                                                     *cameras_[0], *cameras_[1],
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

    cal.F = f;
    cal.E = e;
    cal.R = r;
    cal.t = t;

    cout << "Reprojection error: " << reprojError << endl;
  }


  bool hartleyMethod( FlatCalibrationData &normData,
                     StereoCalibration &cal )
  {
    cout << "!!! Using Hartley method !!!" << endl;

    // Hartley method calculates F directly.  Then decomposes that into E and decomposes
    // that into T and R
    //
    // In this case, assume the cameras are calibrated properly, find E directly, then
    // generate F, T, R
    //
    // NEEDS a single vector of undistorted points
    // Make a set of flattened, undistorted points
    //  and a set of flattened, undistorted, normalized points

    Mat e,f, status;

    //Mat status, estF;
    //estF = findFundamentalMat(Mat(undistortedImagePts[0]), Mat(undistortedImagePts[1]), FM_RANSAC, 3., 0.99, status);
    //cout << "Est F: " << endl << estF << endl;

    Mat estE;
    estE = findFundamentalMat(Mat(normData.imagePoints_[0]),
                              Mat(normData.imagePoints_[1]),
                              FM_RANSAC, 3./1600.0, 0.99, status);

  if( estE.empty() ) {
    LOG(ERROR) << "Could not calculate essential matrix.";
    return -1;
  }

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
    vector<bool> statusVec( status.size().area(), false );
    for( int i = 0; i < status.size().area(); ++i )
      if( status.at<unsigned int>(i,0) > 0 ) {
        statusVec[i] = true;
        ++count;
      }
    cout << count << "/" << normData.size() << " points considered inlier." << endl;

    FlatCalibrationData imagedData;
    normData.normalize( imagedData, *cameras_[0], *cameras_[1] );

    FlatCalibrationData goodNorm, goodImaged;
    normData.keepIf( goodNorm, statusVec );
    imagedData.keepIf( goodImaged, statusVec );

    double eError = reprojectionError( goodNorm, e );
    cout << "Mean E reproj error " << eError << endl;

    // Calculate f
    f = cameras_[1]->mat().inv().t() * e * cameras_[0]->mat().inv();
    f /= (f.at<double>(2,2) == 0 ? 1.0 : f.at<double>(2,2) );

    // Calculate the mean reprojection error under this f
    double fError = reprojectionError( goodImaged, f );
    cout << "Mean F reproj error " << fError << endl;

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

      Point3d x1( goodNorm.imagePoints_[0][0][0], goodNorm.imagePoints_[0][0][1], 1.0 ),
              x2( goodNorm.imagePoints_[1][0][0], goodNorm.imagePoints_[1][0][1], 1.0 );

      Mat X1 = Mx * Mat(x1),
          X2 = Mx * rcand.t() * Mat(x2);

      if( (X1.at<double>(2) * X2.at<double>(2)) < 0 )
        rcand = u * W.t() * vt;
      else if (X1.at<double>(2) < 0)
        tcand *= -1;
      else
        done = true;
    }


    cal.F = f;
    cal.E = e;
    cal.R = rcand;
    cal.t = tcand;

    return true;

    //Mat H[2];
    //stereoRectifyUncalibrated(Mat(allimgpt[0]), Mat(allimgpt[1]), e, imageSize, H[0], H[1], 3);

    //for( int k = 0; k < 2; ++k ) {
    //  R[k] = cam[k].inv()*H[k]*cam[k];
    //  P[k] = cam[k];
    //}
  }


  double reprojectionError( FlatCalibrationData &data, Mat &e )
  {

    // Calculate the mean reprojection error under this f
    double error = 0;
    for( size_t i = 0; i < data.size(); ++i ) {
      Mat err( Mat(Vec3d( data.imagePoints_[1][i][0], data.imagePoints_[1][i][1], 1.0 )).t() *
              e * Mat(Vec3d( data.imagePoints_[0][i][0], data.imagePoints_[0][i][1], 1.0 ) ) );

      double f = err.at<double>(0,0);

      error += f*f;
    }

    error /= data.size();
    return error;
  }




 private:

  DbStereoCalibrationOpts &opts_;
  DistortionModel *cameras_[2];
  DetectionDb db_[2];

};


int main( int argc, char** argv )
{
  google::InitGoogleLogging( argv[0] );
  FLAGS_logtostderr = 1;

  DbStereoCalibrationOpts opts;
  if( !opts.parseOpts( argc, argv ) ) exit(-1);

  DbStereoCalibration main( opts );

  exit( main.run() );

}


#if 0


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



static string mkDbStereoPairFileName( const string &cam0, const string &cam1 )
{
  char strtime[32], filename[80];
  time_t tt;
  time( &tt );
  struct tm *t2 = localtime( &tt );
  strftime( strtime, 32, "%y%m%d_%H%M%S", t2 );
  snprintf( filename, 80, "cal_%s_%s_%s.yml", cam0.c_str(), cam1.c_str(), strtime );
  return  string( filename );
}


void saveDbStereoCalibration( const string &filename,
                             const DbStereoCalibration &cal,
                             const DbStereoRectification &rect,
                             const string &cam0, const string &cam1 )
{

  FileStorage fs( filename, FileStorage::WRITE );

  time_t tt;
  time( &tt );
  struct tm *t2 = localtime( &tt );
  char buf[1024];
  strftime( buf, sizeof(buf)-1, "%c", t2 );

  fs << "calibration_time" << buf;
  fs << "calibration_0" << cam0;
  fs << "calibration_1" << cam1;

  cal.save( fs );
  rect.save( fs );

  cout << "Wrote stereo calibration to " << filename << endl;
}





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
      CompositeCanvas canvas( thisPair[0].img(), thisPair[1].img(), false );

      for( int imgIdx = 0; imgIdx < 2 ; ++imgIdx )
        cameras[imgIdx]->undistortImage( thisPair[imgIdx].img(), canvas.roi[imgIdx] );

      for( int j = 0; j < shared.worldPoints.size(); ++j ) {
        //cout << shared.worldPoints[j] << undistortedPoints[0][j] << undistortedPoints[1][j] << endl;
        //cout << shared.worldPoints[j] << shared.aPoints[j] << shared.bPoints[j] << endl;
        float i = (float)j / shared.worldPoints.size();
        Scalar color( 255*i, 0, (1-i)*255);

        cv::circle( canvas[0], Point(undistortedImagePoints[0].back()[j]), 5, color, -1 );
        cv::circle( canvas[1], Point(undistortedImagePoints[1].back()[j]), 5, color, -1 );

        cv::line(  canvas, Point(undistortedImagePoints[0].back()[j]),
                 canvas.origin(1) + Point(undistortedImagePoints[1].back()[j]),
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

DbStereoCalibration cal;
if( method == HARTLEY ) {
  hartleyMethod( undistortedImagePts, cameras, cal );
} else if( method == STEREO_CALIBRATE ) {
} else {
  cout << "No method specified.  Stopping..." << endl;
  exit(0);
}


// == Attempt to rectify images

Mat R[2], P[2], disparity;
Rect validROI[2];
float alpha = -1;

Distortion::stereoRectify( *cameras[0], *cameras[1], imageSize, cal.R, cal.t,
                          R[0], R[1], P[0], P[1],  disparity, CALIB_ZERO_DISPARITY,
                          alpha, imageSize, validROI[0], validROI[1] );

DbStereoRectification rect;
rect.R[0] = R[0];
rect.R[1] = R[1];
rect.P[0] = P[0];
rect.P[1] = P[1];

saveDbStereoCalibration( opts.stereoPairPath( mkDbStereoPairFileName( opts.cameraName[0], opts.cameraName[1] ) ),
                        cal, rect, cameraCalibrationFiles[0], cameraCalibrationFiles[1] );

cout << "R: " << endl << cal.R << endl;
cout << "T: " << endl << cal.t << endl;
cout << "norm(T): " << norm(cal.t, NORM_L2 ) << endl;
cout << "E: " << endl << cal.E << endl;
cout << "F: " << endl << cal.F << endl;


//Mat H[2];
//stereoRectifyUncalibrated( undistortedImagePts[0], undistortedImagePts[1], cal.F, imageSize, H[0], H[1] );
//cout << "H[0]: " << endl << H[0] << endl;
//cout << "H[1]: " << endl << H[1] << endl;

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
  //cameras[k]->initUndistortRectifyMap( H[k], cameras[k]->mat(), //P[k],
  //    imageSize, CV_32FC1, map[k][0], map[k][1] );
  cameras[k]->initUndistortRectifyMap( R[k], P[k],
                                      imageSize, CV_32FC1, map[k][0], map[k][1] );

  //cout << "map" << k << "0: " << endl << map[k][0] << endl;
  //cout << "map" << k << "1: " << endl << map[k][1] << endl;
}

for( int i = 0; i < pairs.size(); ++i ) {

  ImagePair &thisPair( pairs[i] );
  CompositeCanvas canvas( thisPair[0].img(), thisPair[1].img(), false );

  for( int idx = 0; idx < 2; ++idx ) {
    Mat undist;
    cameras[idx]->undistortImage( thisPair[idx].img(), undist );
    //warpPerspective( undist, canvas.roi[idx], H[idx], imageSize );

    remap( thisPair[idx].img(), canvas.roi[idx], map[idx][0], map[idx][1], INTER_LINEAR );

    Scalar roiBorder( 0,255,0 );
    line( canvas.roi[idx], Point(validROI[idx].x, validROI[idx].y), Point(validROI[idx].x+validROI[idx].width, validROI[idx].y), roiBorder, 1 );
    line( canvas.roi[idx], Point(validROI[idx].x+validROI[idx].width, validROI[idx].y), Point(validROI[idx].x+validROI[idx].width, validROI[idx].y+validROI[idx].height), roiBorder, 1 );
    line( canvas.roi[idx], Point(validROI[idx].x+validROI[idx].width, validROI[idx].y+validROI[idx].height), Point(validROI[idx].x, validROI[idx].y+validROI[idx].height), roiBorder, 1 );
    line( canvas.roi[idx], Point(validROI[idx].x, validROI[idx].y+validROI[idx].height), Point(validROI[idx].x, validROI[idx].y), roiBorder, 1 );

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
//          rectifiedPoints[j].begin(), RectifierFunctor( cam*R[j] ) );
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
  cal.R.copyTo( subR );
  cal.t.copyTo( subT );

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

#endif
