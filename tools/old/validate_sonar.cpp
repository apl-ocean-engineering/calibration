
#include <iostream>
#include <fstream>
#include <string>

#include <glog/logging.h>
#include <tclap/CmdLine.h>


#include "distortion/camera_factory.h"
#include "sonar_pose.h"
#include "stereo_calibration.h"

using namespace std;
using namespace cv;
using namespace Distortion;


class ValidatorOpts {
public:
  ValidatorOpts( void )
  {;}

  string leftCameraCal, rightCameraCal, stereoCal, leftCameraSonar, rightCameraSonar;
  string sonarDetections, cameraDetections;
  bool imageAxes;

  bool parseCmdLine( int argc, char **argv )
  {

    try {
      TCLAP::CmdLine cmd("Visualizers", ' ', "0.1" );

      TCLAP::ValueArg< string > cameraFileArg("", "camera-detections", "Camera detection file", false, "", "Detections file", cmd );
      TCLAP::ValueArg< string > sonarFileArg("", "sonar-detections", "sonar detection file", false, "", "Detections file", cmd );

      TCLAP::ValueArg< string > leftCameraCalArg("", "left-camera", "Camera calibration", false, "", "Calibration file", cmd );
      TCLAP::ValueArg< string > rightCameraCalArg("", "right-camera", "Camera calibration", false, "", "Calibration file", cmd );

      TCLAP::ValueArg< string > stereoCalArg("", "stereo-calibration", "Camera calibration", true, "", "Calibration file", cmd );

      TCLAP::ValueArg< string > leftCameraSonarArg("", "left-camera-sonar", "Camera-sonar calibration", true, "", "Calibration file", cmd );
      TCLAP::ValueArg< string > rightCameraSonarArg("", "right-camera-sonar", "Camera-sonar calibration", true, "", "Calibration file", cmd );


      TCLAP::SwitchArg imgAxesArg( "", "use-image-axes", "Image axes", cmd, false );

      cmd.parse( argc, argv );

      leftCameraCal = leftCameraCalArg.getValue();
      rightCameraCal = rightCameraCalArg.getValue();

      leftCameraSonar = leftCameraSonarArg.getValue();
      rightCameraSonar = rightCameraSonarArg.getValue();

      stereoCal = stereoCalArg.getValue();

      cameraDetections   = cameraFileArg.getValue();
      sonarDetections    = sonarFileArg.getValue();

      imageAxes = imgAxesArg.getValue();


    } catch (TCLAP::ArgException &e) {
      LOG(ERROR) << "error: " << e.error() << " for arg " << e.argId();
    }

    return validate();
  }

  bool  validate( void )
  {
    return true;
  }

};


class Validator {
public:

  Validator( ValidatorOpts &opts_ )
  : opts(opts_), _leftCam( NULL), _rightCam( NULL ), _leftCS( NULL ), _rightCS( NULL )
  {;}

  ~Validator()
  {
    if( _leftCam != NULL ) delete _leftCam;
    if( _rightCam != NULL ) delete _rightCam;
    if( _leftCS != NULL ) delete _leftCS;
    if( _rightCS != NULL ) delete _rightCS;
  }

  int run( void )
  {
    int ret = loadCalibrations();
    if( ret != 0 ) return ret;

// Ground truths
float leftCalcLength = 865.570874084302, rightCalcLength = 862.4962716725214, stereoBaseline = 743.74375;

    // This is the rigid body transform from sonar frame to right from to left frame
    Matx33f stereoR;
    Vec3f   stereot;

    _stereo.R.convertTo( stereoR, CV_32F );
    _stereo.t.convertTo( stereot, CV_32F );

    LOG(INFO) << "-----  stereo ----";
    LOG(INFO) << "Rot: " << euler( stereoR );
    LOG(INFO) << "Trans: " << stereot;
float sb = sqrt( stereot.dot( stereot ));
    LOG(INFO) << "Baseline length: " << sb;
LOG(INFO) << "Ground truth baseline: " << stereoBaseline;
LOG(INFO) << "Baseline error: " << (fabs(sb-stereoBaseline)/stereoBaseline) * 100 << " pct";
    LOG(INFO) << endl;

    LOG(INFO) << "---- Left pose ---- ";
    // This is the rigid body transform from sonar frame to left cam frame
    LOG(INFO) << "Rot: " << euler(_leftCS->rotMat());
    LOG(INFO) << "Trans: " << _leftCS->trans();
    LOG(INFO) << "Trans length: " << _leftCS->tLength();
LOG(INFO) << "Ground truth length: " << leftCalcLength;

LOG(INFO) << "Error : " << (fabs(_leftCS->tLength() - leftCalcLength)/leftCalcLength) * 100 << " pct";

    Matx33f txRot = stereoR.t() * _rightCS->rotMat();
    Vec3f   txTrans = stereoR.t() * (_rightCS->trans() - stereot );

    LOG(INFO) << "----- Right + stereo ----";
    LOG(INFO) << "Rot: " << euler( txRot );
    LOG(INFO) << "Trans: " << txTrans;
float rightStLength = sqrt( txTrans.dot(txTrans ));
    LOG(INFO) << "Trans length: " << rightStLength;
LOG(INFO) << "Error : " << (fabs(_leftCS->tLength() - rightStLength)/_leftCS->tLength()) * 100 << " pct";

    Vec3f terror = _leftCS->trans() - txTrans;
    LOG(INFO) << endl;
    LOG(INFO) << "Left to right+stereo Trans error: " << terror;
    LOG(INFO) << "   trans error length: " << sqrt( terror.dot(terror));

Vec3f crossProd( _leftCS->trans().cross( txTrans ) );
float cross = sqrt( crossProd.dot(crossProd ));
float dot = _leftCS->trans().dot( txTrans );
    float leftErrAngle = atan2( cross, dot );
    LOG(INFO) << "Included angle: " << 180.0 / M_PI * leftErrAngle;
    LOG(INFO) << endl;

    LOG(INFO) << "---- Right pose ---- ";
    // This is the rigid body transform from sonar frame to left cam frame
    LOG(INFO) << "Rot: " << euler(_rightCS->rotMat());
    LOG(INFO) << "Trans: " << _rightCS->trans();
    LOG(INFO) << "Trans length: " << _rightCS->tLength();
    LOG(INFO) << "Ground truth length: " << rightCalcLength;

LOG(INFO) << "Error: " << (fabs(_rightCS->tLength() - rightCalcLength)/rightCalcLength) * 100 << " pct";

    txRot = stereoR * _leftCS->rotMat();
    txTrans = stereoR * (_leftCS->trans() + stereot);

    LOG(INFO) << "----- Left + stereo ----";
    LOG(INFO) << "Rot: " << euler( txRot );
    LOG(INFO) << "Trans: " << txTrans;
    float leftStLength = sqrt( txTrans.dot(txTrans ));
        LOG(INFO) << "Trans length: " << leftStLength;
    LOG(INFO) << "Error : " << (fabs(_rightCS->tLength() - leftStLength)/_rightCS->tLength()) * 100 << " pct";

    terror = _rightCS->trans() - txTrans;
    LOG(INFO) << endl;
    LOG(INFO) << "Left to right+stereo Trans error: " << terror;
    LOG(INFO) << "   trans error length: " << sqrt( terror.dot(terror));

     crossProd = _rightCS->trans().cross( txTrans );
     cross = sqrt( crossProd.dot(crossProd ));
     dot = _rightCS->trans().dot( txTrans );
        float rightErrAngle = atan2( cross, dot );
    LOG(INFO) << "Included angle: " << 180.0 / M_PI * rightErrAngle;

    LOG(INFO) << endl;


    return 0;

  }

  int loadCalibrations( void )
  {
    _leftCam = CameraFactory::LoadDistortionModel( opts.leftCameraCal );
    if( _leftCam == NULL ) {
      LOG(ERROR) << "Couldn't load left camera cal from " << opts.leftCameraCal;
      return -1;
    }

    _rightCam = CameraFactory::LoadDistortionModel( opts.rightCameraCal );
    if( _rightCam == NULL ) {
      LOG(ERROR) << "Couldn't load right camera cal from " << opts.rightCameraCal;
      return -1;
    }

    _leftCS = SonarPose::Load( opts.leftCameraSonar);
    if( _leftCS == NULL ) {
      LOG(ERROR) << "Couldn't load left camera-sonar cal from " << opts.leftCameraSonar;
      return -1;
    }

    _rightCS = SonarPose::Load( opts.rightCameraSonar);
    if( _rightCS == NULL ) {
      LOG(ERROR) << "Couldn't load right camera-sonar cal from " << opts.rightCameraSonar;
      return -1;
    }

    if( _stereo.load( opts.stereoCal ) == false ) {
      LOG(ERROR) << "Couldn't load stereo calibration from " << opts.stereoCal;
      return -1;
    };

    return 0;
  }

  Vec3f euler( const Matx33f &rot ) const
  {
    cv::Matx33f qx, qy, qz, Rq, Qq;
    cv::RQDecomp3x3( rot, Rq, Qq, qx, qy, qz );

    return Vec3f( acos( qx(1,1) ) * 180.0/M_PI,
    acos( qy(0,0) ) * 180.0/M_PI,
    acos( qz(0,0) ) * 180.0/M_PI);
  }

protected:

  ValidatorOpts &opts;

  DistortionModel *_leftCam, *_rightCam;
  SonarPose *_leftCS, *_rightCS;
  StereoCalibration _stereo;
};



// --------------
// -----Main-----
// --------------
int main (int argc, char** argv)
{
  google::InitGoogleLogging( argv[0] );
  FLAGS_logtostderr = 1;


  ValidatorOpts opts;
  if( !opts.parseCmdLine( argc, argv ) ) exit(-1);

  Validator viz( opts );
  return viz.run();
}
