#include <string>

#include <tclap/CmdLine.h>
#include <glog/logging.h>

using namespace std;


class SonarCalibrationOpts {
  public:
    SonarCalibrationOpts( void )
    {;}

    string sonarFile, sphereDb;

    bool parseOpts( int argc, char **argv )
    {

      try {
        TCLAP::CmdLine cmd("Attempt video-sonar calibration", ' ', "0.1" );

        TCLAP::ValueArg<std::string> sonarFileArg("", "sonar-file", "Sonar file", true, "", "Sonar file", cmd );
        TCLAP::ValueArg<std::string> sphereDbArg("", "sphere-db", "Sphere db", true, "", "Sphere db", cmd );

        cmd.parse( argc, argv );

        sonarFile = sonarFileArg.getValue();
        sphereDb = sphereDbArg.getValue();

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


int main( int argc, char** argv )
{
  google::InitGoogleLogging( argv[0] );
  FLAGS_logtostderr = 1;

  SonarCalibrationOpts opts;
  if( !opts.parseOpts( argc, argv ) ) exit(-1);





}

