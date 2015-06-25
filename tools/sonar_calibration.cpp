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

using namespace std;
using namespace Eigen;

using namespace Distortion;


class SonarCalibrationOpts {
 public:
  SonarCalibrationOpts( void )
  {;}

  string sonarFile, cameraFile, cameraCalibration;

  bool parseOpts( int argc, char **argv )
  {

    try {
      TCLAP::CmdLine cmd("Attempt video-sonar calibration", ' ', "0.1" );

      TCLAP::ValueArg<std::string> sonarFileArg("", "sonar-file", "Sonar file", true, "", "Sonar file", cmd );
      TCLAP::ValueArg<std::string> cameraFileArg("", "camera-file", "Sphere db", true, "", "Sphere db", cmd );
      TCLAP::ValueArg<std::string> cameraCalArg("", "camera-calibration", "Camera cal", true, "", "Camera calibration file", cmd );

      cmd.parse( argc, argv );

      sonarFile = sonarFileArg.getValue();
      cameraFile = cameraFileArg.getValue();
      cameraCalibration = cameraCalArg.getValue();

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

    vector<string> tokens;
    string sline, vline;
    while( std::getline( son, sline ) ) {
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
      while( !stop and std::getline( vid, vline ) ) {
        boost::split(tokens, vline, boost::is_any_of(","));

        if( tokens.size() < 3 ) continue;

        if( tokens[0] == fname ) {
          LOG(INFO) << "Match: " << fname << " " << tokens[0];

          Vector2f vPoint( atof( tokens[1].c_str() ),
                          atof( tokens[2].c_str() ) );

          data.push_back( SonarCalibrationSolver::SonarCalDatum( vPoint, sPoint, fname ) );
          stop = true;
        }
      }

    }

    LOG(INFO) << "Loaded " << data.size() << " points";



    SonarCalibrationSolver solver;
    solver.solve( data );


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

