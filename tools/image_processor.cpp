
#include <stdlib.h>
#include <stdio.h>

#include <iostream>
#include <iomanip>
#include <string>
#include <sstream>
#include <fstream>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <getopt.h>

#include <tclap/CmdLine.h>
#include <glog/logging.h>

#include "file_utils.h"
#include "composite_canvas.h"
#include "stereo_calibration.h"

#include "distortion/distortion_model.h"
#include "distortion/distortion_stereo.h"
#include "distortion/camera_factory.h"
using namespace Distortion;

#include "video_prefs.h"

#include <boost/filesystem.hpp>

using namespace cv;
using namespace std;

using namespace boost::filesystem;

//using AprilTags::TagDetection;


using namespace AplCam;

struct Options
{

  typedef enum { UNDISTORT,  VERB_NONE = -1 } Verb;

  // n.b. the default window should be a non-round number so you don't get an ambiguous number of second transitions...
  Options()
  : verb( VERB_NONE )

  {;}


  Verb verb;
  string cameraCalibration,  outputFile;
  vector<string> inputFiles;
  bool doDisplay;

  bool parseOpts( int argc, char **argv, stringstream &msg )
  {

    try {

      TCLAP::CmdLine cmd("image_processor", ' ', "0.1");

      TCLAP::SwitchArg doDisplayArg("X", "do-display", "display", cmd, false );

      TCLAP::ValueArg<std::string> calArg("c","camera-calibration", "Calibration file basename", true, "", "name", cmd );
      TCLAP::ValueArg<std::string> outputArg("o", "output", "Output file", false, "", "file name", cmd );


      //TCLAP::SwitchArg doAnnotateArg("N", "do-annotate", "Do annotate entries into a video", cmd, false );

      TCLAP::UnlabeledValueArg< std::string > verbArg( "verb", "Verb", true, "", "verb", cmd );
      TCLAP::UnlabeledMultiArg< std::string > inputFilesArg( "input-files", "Input files", true, "file name", cmd );

      cmd.parse( argc, argv );

      doDisplay = doDisplayArg.getValue();

      outputFile = outputArg.getValue();
      inputFiles = inputFilesArg.getValue();
      cameraCalibration = calArg.getValue();

      string v = verbArg.getValue();
      if( v == "undistort" ) {
        verb = UNDISTORT;
      } else {
        LOG(ERROR) << "Didn't understand verb \"" << v << "\"";
        return false;
      }


    } catch( TCLAP::ArgException &e ) {
      LOG( ERROR ) << "error: " << e.error() << " for arg " << e.argId();
    }

    return validate( msg );
  }

  bool validate( stringstream &msg )
  {
    path outPath( outputFile );
    if ((inputFiles.size() > 1) && (is_directory(outPath) == false) ) {
      LOG(ERROR) << "If more than one input file is specified, the output file must be a directory";
      return false;
    }

if( !is_regular_file( path( cameraCalibration))) {
LOG(ERROR) << "Camera calibration file \"" << cameraCalibration << "\" doesn't exist.";
return false;
}

    return true;
  }

  string makeOutputFile( const string &inputFile )
  {
    path outPath( outputFile );

    if( is_directory(outPath) ) {
      return (outPath.append( path(inputFile).string())).string();
    }

    return outputFile;

  }

};


class ImageProcessorMain {
public:
  ImageProcessorMain( Options &opt )
  : opts( opt )
  {;}

  int go( void )
  {
    if( loadCalibrationFiles() == false ) return -1;

    for( size_t i = 0; i < opts.inputFiles.size(); ++i ) {

      switch( opts.verb ) {
        case Options::UNDISTORT:
        return doUndistort( opts.inputFiles[i], opts.makeOutputFile( opts.inputFiles[i] ));
        break;
        case Options::VERB_NONE:
        default:
        LOG(ERROR) << "No verb selected, oh well." << endl;
        return 0;
      }
    }



    return 0;
  }

private:
  Options &opts;

  Mat map[2];

  DistortionModel *camera;



  int doUndistort( const string &in, const string &out )
  {

    Mat img = imread( in );
    Mat undistorted;

    camera->undistortImage( img, undistorted, camera->mat(), img.size() );

    imwrite( out, undistorted );

    return 0;
  }




  bool loadCalibrationFiles( void )
  {

    camera = CameraFactory::LoadDistortionModel( opts.cameraCalibration );

    if( camera== NULL) {
      cerr << "Error loading calibration file " + opts.cameraCalibration << endl;
      return false;
    }

    return true;
  }


};


int main( int argc, char **argv )
{
  google::InitGoogleLogging( argv[0] );
  FLAGS_logtostderr = 1;

  Options opts;
  stringstream msg;
  if( !opts.parseOpts( argc, argv, msg ) ) {
    cout << msg.str() << endl;
    exit(-1);
  }

  ImageProcessorMain main( opts );

  exit( main.go() );
}
