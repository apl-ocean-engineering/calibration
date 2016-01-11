
#include <fstream>
#include <glog/logging.h>

#include "detection_io.h"


namespace CameraCalibration {

using namespace std;
using namespace cv;
using namespace AplCam;

DetectionIO::DetectionIO( void )
{ ; }

DetectionIO::~DetectionIO( void )
{ ; }

DetectionIO *DetectionIO::Create( fs::path &path )
{
  if( fs::is_directory(path) ) {
    return new DetectionIODir( path );
  } else {
    LOG(FATAL) << "Don't know what to do with detection destination \"" << path.string() << "\"";
  }

  LOG(FATAL) << "Should never get here.";
  return NULL;
  // else if( path.extension() ==  ".kch" ) { }
}



//==============================================================
// DetectionIODir

DetectionIODir::DetectionIODir( const fs::path &path )
  : DetectionIO(), _path( path )
{ ; }

DetectionIODir::~DetectionIODir( void )
{ ; }

bool DetectionIODir::save( const string &name, Detection *detection )
{
  fs::path filename( _path );
  filename /= name;
  filename.replace_extension( ".yml" );

  LOG(INFO) << "Saving detections to \"" << filename.string() << "\"";

  string data(detection->serialize());

  ofstream of( filename.string() );
  if( !of.is_open() ) {
    LOG(INFO) << "Unable to open filename \"" << filename.string() << "\"";
    return false;
  }
  of << data;

  return true;
}


}
