

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

#include <glog/logging.h>
#include <tclap/CmdLine.h>

#include <boost/filesystem/path.hpp>

#include "video_prefs.h"

using namespace cv;
using namespace std;

namespace fs = boost::filesystem;

struct Options
{

  Options()
  {;}

  string outputFile;


  bool parseOpts( int argc, char **argv )
  {

    try {

      TCLAP::CmdLine cmd("test_video", ' ', "0.1");

      TCLAP::UnlabeledValueArg< std::string > outputFileArg( "output-file", "Output file", true, "", "file name", cmd );

      cmd.parse( argc, argv );

      outputFile = fs::path(outputFileArg.getValue()).replace_extension( VideoExtension ).string();

    } catch( TCLAP::ArgException &e ) {
      LOG( ERROR ) << "error: " << e.error() << " for arg " << e.argId();
    }

    return validate( );
  }

  bool validate( void)
  {
    return true;
  }

};



int main( int argc, char **argv )
{
  google::InitGoogleLogging( argv[0] );
  FLAGS_logtostderr = 1;
  Options opts;

  if( !opts.parseOpts( argc, argv ) ) exit(-1);

  VideoWriter writer;
  Size outputSize( 1920, 1080 );
  float fps = 29.97;

  writer.open( opts.outputFile, VideoCodec, fps, outputSize );

  if( !writer.isOpened() ) {
    LOG(ERROR) << "Error creating writer for " << opts.outputFile;
    return -1;
  }

  Mat channels[3] = { Mat( outputSize, CV_8UC1 ), Mat(outputSize, CV_8UC1), Mat(outputSize, CV_8UC1) },
  frame( outputSize, CV_8UC3 );

  //for( unsigned int f = 0; f < 3; ++f ) channels[f].create( outputSize, CV_8UC1 );

  for( unsigned int i = 0; i < 1200; ++i ) {

    for( unsigned int f = 0; f < 3; ++f ) randu( channels[f], 0, 255 );

    merge( channels, 3, frame );

    if( (i % 50) == 0 ) LOG(INFO) << "Writing frame " << i;
    writer << frame;
  }

  writer.release();

  return 0;
}
