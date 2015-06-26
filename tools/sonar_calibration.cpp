#include <string>
#include <vector>
#include <fstream>

#include <tclap/CmdLine.h>
#include <glog/logging.h>

#include <Eigen/Core>

#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/split.hpp>

#include "camera_factory.h"

#include "sonar_calibration_solver.h"

#include "sonar_pose.h"

using namespace std;
using namespace Eigen;

using namespace Distortion;


class SonarCalibrationOpts {
 public:
  SonarCalibrationOpts( void )
  {;}

  string sonarFile, cameraFile, cameraCalibration, calOut;

  bool parseOpts( int argc, char **argv )
  {

    try {
      TCLAP::CmdLine cmd("Attempt video-sonar calibration", ' ', "0.1" );

      TCLAP::ValueArg<std::string> sonarFileArg("", "sonar-file", "Sonar file", true, "", "Sonar file", cmd );
      TCLAP::ValueArg<std::string> cameraFileArg("", "camera-file", "Sphere db", true, "", "Sphere db", cmd );
      TCLAP::ValueArg<std::string> cameraCalArg("", "camera-calibration", "Camera cal", true, "", "Camera calibration file", cmd );

      TCLAP::ValueArg<std::string> calOutArg("", "calibration-out", "Out", false, "", "Save calibration", cmd );

      cmd.parse( argc, argv );

      sonarFile = sonarFileArg.getValue();
      cameraFile = cameraFileArg.getValue();
      cameraCalibration = cameraCalArg.getValue();
      calOut = calOutArg.getValue();

    } catch( TCLAP::ArgException &e ) {
      LOG(ERROR) << "Parsing error: " << e.error() << " for " << e.argId();
    }

    return validate( );
  }

  bool validate( void )
  {

    return true;
  }


};



class SonarCalibration {
 public:
  SonarCalibration( SonarCalibrationOpts &options )
      : opts( options )
  {;}

  int run( void )
  {

    camera = CameraFactory::LoadDistortionModel( opts.cameraCalibration );
    if( camera == NULL ) {
      LOG(ERROR) << "Unable to load camera calibration \"" << opts.cameraCalibration << "\"";
      return -1;
    }

    // Load data from sonar and video files
    // Do this in an inefficient way for now
    ifstream son( opts.sonarFile );
    if( !son.is_open() ) {
      LOG(ERROR) << "Unable to open sonar file \"" << opts.sonarFile << "\"";
      return -1;
    }

    ifstream vid( opts.cameraFile );
    if( !vid.is_open() ) {
      LOG(ERROR) << "Unable to open video file \"" << opts.cameraFile << "\"";
      return -1;
    }

    string sline, vline;
    vector<string> vLines;
    while( std::getline( vid, vline ) ) vLines.push_back( vline );

    int sCount = 0;
    vector<string> tokens;
    while( std::getline( son, sline ) ) {
      sCount++;
      boost::split(tokens, sline, boost::is_any_of(","));

      if( tokens.size() < 4 ) {
        LOG(INFO) << "Malformed sonar line: " << sline;
        continue;
      }

      string fname( tokens[0] );
      Vector3f  sPoint( atof( tokens[1].c_str() ), 
                       atof( tokens[2].c_str() ),
                       atof( tokens[3].c_str() ) );

      // Rewind vid
      vid.seekg( 0 );
      bool stop = false;
      for( size_t i = 0; i < vLines.size() and !stop; ++i ) {
        boost::split(tokens, vLines[i], boost::is_any_of(","));

        if( tokens.size() < 3 ) continue;

        if( tokens[0].compare( 0, 18, fname ) == 0 ) {
          LOG(INFO) << "Match: " << fname << " " << tokens[0];

          ImagePoint vImage( atof( tokens[1].c_str() ),
                          atof( tokens[2].c_str() ) );
          float radius( atof( tokens[3].c_str() ) );

          // Normalize and undistort vPoint
          ImagePoint undistorted = camera->undistort( vImage, false );

          Vector2f undVec( undistorted[0], undistorted[1] );

          data.push_back( SonarCalibrationSolver::SonarCalDatum( undVec, sPoint, fname ) );
          stop = true;
        }
      }

    }

    LOG(INFO) << "Loaded " << data.size() << " points from " << sCount << " sonar points and " << vLines.size() << " video points";

    const int minDataPoints = 3;
    if( data.size() < minDataPoints ) {
      LOG(ERROR) << "Insufficient points to solve.";
      return -1;
    }



    SonarCalibrationSolver solver;
    SonarCalibrationSolver::Result result;
    solver.solve( data, result );

    LOG(INFO) << "Solution is: " << (result.good ? "GOOD" : "BAD");
    if( result.good ) {
      SonarPose pose( result.pose );

      LOG(INFO) << "Rotation vector: " << pose.rot();
      Vec3f euler( pose.euler() );
      LOG(INFO) << euler;
      for( int i = 0; i < 3; ++i )
        euler[i] *= 180.0/M_PI;

      LOG(INFO) << "Euler angles: " << euler;

      LOG(INFO) << "Translation vector: " << pose.trans();

      if( opts.calOut.size() > 0 ) pose.write( opts.calOut );

    }


    return 0;
  }


 protected:

  SonarCalibrationSolver::SonarCalData data;

  DistortionModel *camera;
  SonarCalibrationOpts &opts;
};

int main( int argc, char** argv )
{
  google::InitGoogleLogging( argv[0] );
  FLAGS_logtostderr = 1;

  SonarCalibrationOpts opts;
  if( !opts.parseOpts( argc, argv ) ) exit(-1);

  SonarCalibration cal( opts );
  return cal.run();
}

