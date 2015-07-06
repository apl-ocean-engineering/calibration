#include <string>
#include <vector>
#include <fstream>

#include <tclap/CmdLine.h>
#include <glog/logging.h>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include <boost/filesystem/path.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/split.hpp>

#include "distortion/camera_factory.h"

using namespace std;
using namespace cv;

using namespace Distortion;


class AnnotateSphereOpts {
public:
  AnnotateSphereOpts( void )
  {;}

  string cameraFile, cameraCalibration, imageFile, outputFile;
  bool imageAxes;

  bool parseOpts( int argc, char **argv )
  {

    try {
      TCLAP::CmdLine cmd("Attempt video-sonar calibration", ' ', "0.1" );

      TCLAP::ValueArg<std::string> cameraFileArg("", "camera-file", "Sphere db", true, "", "Sphere db", cmd );
      TCLAP::ValueArg<std::string> cameraCalArg("", "camera-calibration", "Camera cal", true, "", "Camera calibration file", cmd );
      TCLAP::ValueArg<std::string> outputFileArg("o", "output-file", "Output file", true, "", "Output file", cmd );

      TCLAP::ValueArg<std::string> imageFileArg("i", "image-file", "Image file", true, "", "Image file", cmd);

      cmd.parse( argc, argv );


      imageFile = imageFileArg.getValue();
      cameraFile = cameraFileArg.getValue();
      cameraCalibration = cameraCalArg.getValue();
      outputFile = outputFileArg.getValue();

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



class AnnotateSphere {
public:
  AnnotateSphere( AnnotateSphereOpts &options )
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
    ifstream vid( opts.cameraFile );
    if( !vid.is_open() ) {
      LOG(ERROR) << "Unable to open video file \"" << opts.cameraFile << "\"";
      return -1;
    }

    // Get the root of the image filename
    string timestamp( boost::filesystem::path( opts.imageFile ).stem().string() );

    LOG(INFO) << "Looking for timestamp: " << timestamp;



    string line;

    vector<string> tokens;
    while( std::getline( vid, line ) ) {

      boost::split(tokens, line, boost::is_any_of(","));

      if( tokens.size() < 4 ) {
        LOG(INFO) << "Malformed line in video detection file: " << line;
        continue;
      }

      if( tokens[0].find( timestamp ) != string::npos ) {
        Point2f center( atof( tokens[1].c_str() ), atof( tokens[2].c_str() ) );
        float radius( atof( tokens[3].c_str() ) );

        Mat img = imread( opts.imageFile );
        if( img.empty() ) {
          LOG(ERROR) << "Couldn't load image file";
          return -1;
        }

        cv::circle( img, center, radius, Scalar( 0,0,255 ), 2 );

        imwrite( opts.outputFile, img );
        LOG(INFO) << "Saved output to " << opts.outputFile;

        return 0;
      }


    }

    LOG(ERROR) << "Could not find entry for image " << opts.imageFile;

    return 0;
  }


protected:

  DistortionModel *camera;
  AnnotateSphereOpts &opts;
};

int main( int argc, char** argv )
{
  google::InitGoogleLogging( argv[0] );
  FLAGS_logtostderr = 1;

  AnnotateSphereOpts opts;
  if( !opts.parseOpts( argc, argv ) ) exit(-1);

  AnnotateSphere cal( opts );
  return cal.run();
}
